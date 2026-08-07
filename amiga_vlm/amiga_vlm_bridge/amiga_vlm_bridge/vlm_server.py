import base64
import threading
from typing import Optional

import cv2
import requests
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image

from amiga_vlm_interfaces.srv import VlmAsk


class VlmServer(Node):

    def __init__(self):
        super().__init__("vlm_server")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("service_name", "/vlm/ask")
        self.declare_parameter("vlm_url", "http://localhost:9000/v1/chat/completions")
        self.declare_parameter("system_prompt", "You are a helpful AI assistant.")
        self.declare_parameter("max_tokens", 256)
        self.declare_parameter("min_tokens", 1)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("http_timeout_sec", 180.0)

        self._image_topic: str = self.get_parameter("image_topic").value
        self._service_name: str = self.get_parameter("service_name").value
        self._vlm_url: str = self.get_parameter("vlm_url").value
        self._system_prompt: str = self.get_parameter("system_prompt").value
        self._max_tokens: int = int(self.get_parameter("max_tokens").value)
        self._min_tokens: int = int(self.get_parameter("min_tokens").value)
        self._jpeg_quality: int = int(self.get_parameter("jpeg_quality").value)
        self._http_timeout: float = float(self.get_parameter("http_timeout_sec").value)

        self._bridge = CvBridge()
        self._latest_bgr: Optional["cv2.Mat"] = None
        self._frame_lock = threading.Lock()
        self._infer_lock = threading.Lock()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, self._image_topic, self._on_image, image_qos)
        self.create_service(VlmAsk, self._service_name, self._on_ask)

        self.get_logger().info(f"image_topic={self._image_topic}")
        self.get_logger().info(f"service_name={self._service_name}")
        self.get_logger().info(f"vlm_url={self._vlm_url}")

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        with self._frame_lock:
            self._latest_bgr = bgr

    def _on_ask(self, request: VlmAsk.Request, response: VlmAsk.Response) -> VlmAsk.Response:
        question = (request.question or "").strip()
        if not question:
            response.success = False
            response.answer = ""
            response.error = "Empty question"
            return response

        with self._infer_lock:
            with self._frame_lock:
                frame = None if self._latest_bgr is None else self._latest_bgr.copy()

            if frame is None:
                response.success = False
                response.answer = ""
                response.error = "No image received yet"
                return response

            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
            if not ok:
                response.success = False
                response.answer = ""
                response.error = "Failed to encode image"
                return response

            data_url = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("utf-8")

            payload = {
                "messages": [
                    {"role": "system", "content": self._system_prompt},
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": question},
                            {"type": "image_url", "image_url": {"url": data_url}},
                        ],
                    },
                ],
                "max_tokens": self._max_tokens,
                "min_tokens": self._min_tokens,
            }

            try:
                r = requests.post(self._vlm_url, json=payload, timeout=self._http_timeout)
                r.raise_for_status()
                answer = r.json()["choices"][0]["message"]["content"]
                response.success = True
                response.answer = answer
                response.error = ""
                return response
            except Exception as e:
                response.success = False
                response.answer = ""
                response.error = str(e)
                return response


def main() -> None:
    rclpy.init()
    node = VlmServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
