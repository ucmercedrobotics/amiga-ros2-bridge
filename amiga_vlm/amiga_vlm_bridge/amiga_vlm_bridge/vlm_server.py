import base64
import threading
from typing import Optional

import cv2
import requests
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image

from amiga_vlm_interfaces.srv import VlmAsk


class VlmServer(Node):

    def __init__(self):
        super().__init__("vlm_server")

        # The Oak-D front camera, on the real robot and through the Gazebo
        # shim alike. Was a ZED topic, which nothing in this workspace
        # publishes, so the default could only report "No image received yet".
        self.declare_parameter("image_topic", "/oak0/rgb/image_raw")
        self.declare_parameter("service_name", "/vlm/ask")
        # 8001, not 8000: the agents' reasoning model is what lives on 8000
        # (llm.py's AGENT_API_BASE default), and this is a different model on a
        # different service. A default that collided with it would send camera
        # frames to a text model, or quietly work and put a 4B describer where a
        # reasoning model was meant to be.
        self.declare_parameter("vlm_url", "http://localhost:8001/v1/chat/completions")
        self.declare_parameter("system_prompt", "You are a helpful AI assistant.")
        self.declare_parameter("max_tokens", 256)
        # A vLLM extension, not part of the OpenAI schema. Sent only when set
        # above 0, so the default request is one any compatible server accepts;
        # a stricter one rejects the unknown field and the failure reads as the
        # whole VLM being down.
        self.declare_parameter("min_tokens", 0)
        self.declare_parameter("jpeg_quality", 85)
        # Generous for a hand-driven `ros2 service call` against a cold model.
        # Every automated caller overrides it downward -- the agent launch files
        # pass 6.0, because triage gives up at 8.0 and a reply nobody is still
        # waiting for is worse than an error.
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

        # Separate groups, with a MultiThreadedExecutor in main(): _on_ask
        # blocks on HTTP for seconds, and on a single-threaded executor the
        # frame buffer would stop being refilled for that whole time, so the
        # next question would be answered from a picture taken before the last
        # inference began.
        self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            image_qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.create_service(
            VlmAsk,
            self._service_name,
            self._on_ask,
            callback_group=ReentrantCallbackGroup(),
        )

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
            }
            if self._min_tokens > 0:
                payload["min_tokens"] = self._min_tokens

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
    """Spin until asked to stop, then clean up without a traceback.

    Ctrl-C raises KeyboardInterrupt; `ros2 launch` sending SIGTERM shuts the
    context down underneath us and raises ExternalShutdownException, after
    which an unconditional shutdown() raises again. Same shape as
    amiga_ros2_agents/runtime/spin.py, and now for the same reason: both agent
    launch files start this node.
    """
    rclpy.init()
    node = VlmServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
