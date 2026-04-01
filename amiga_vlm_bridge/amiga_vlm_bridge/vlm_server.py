import base64
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import cv2
import requests
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image

from amiga_vlm_interfaces.srv import VlmLoopStart, VlmLoopStop


class VlmServer(Node):
    """
    Starts a background loop via ``VlmLoopStart``: every ``interval_sec``, sends the latest
    camera frame plus a fixed prompt to a local OpenAI-Vision-compatible HTTP endpoint and
    appends each response to a single ``output_file``.
    ``VlmLoopStop`` ends the loop.
    """

    def __init__(self):
        super().__init__("vlm_server")

        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("loop_start_service", "/vlm/start")
        self.declare_parameter("loop_stop_service", "/vlm/stop")
        self.declare_parameter("default_loop_interval_sec", 2.0)
        self.declare_parameter("vlm_url", "http://localhost:9000/v1/chat/completions")
        self.declare_parameter("system_prompt", "You are a helpful AI assistant.")
        self.declare_parameter("max_tokens", 256)
        self.declare_parameter("min_tokens", 1)
        self.declare_parameter("jpeg_quality", 85)
        self.declare_parameter("http_timeout_sec", 180.0)

        self._image_topic: str = self.get_parameter("image_topic").value
        self._loop_start_name: str = self.get_parameter("loop_start_service").value
        self._loop_stop_name: str = self.get_parameter("loop_stop_service").value
        self._default_interval: float = float(self.get_parameter("default_loop_interval_sec").value)
        self._vlm_url: str = self.get_parameter("vlm_url").value
        self._system_prompt: str = self.get_parameter("system_prompt").value
        self._max_tokens: int = int(self.get_parameter("max_tokens").value)
        self._min_tokens: int = int(self.get_parameter("min_tokens").value)
        self._jpeg_quality: int = int(self.get_parameter("jpeg_quality").value)
        self._http_timeout: float = float(self.get_parameter("http_timeout_sec").value)

        self._bridge = CvBridge()
        self._latest_bgr: Optional["cv2.Mat"] = None
        self._frame_lock = threading.Lock()

        self._loop_stop_event = threading.Event()
        self._loop_thread: Optional[threading.Thread] = None
        self._loop_thread_lock = threading.Lock()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Image, self._image_topic, self._on_image, image_qos)
        self.create_service(VlmLoopStart, self._loop_start_name, self._on_loop_start)
        self.create_service(VlmLoopStop, self._loop_stop_name, self._on_loop_stop)

        self.get_logger().info(f"image_topic={self._image_topic}")
        self.get_logger().info(f"loop_start_service={self._loop_start_name}")
        self.get_logger().info(f"loop_stop_service={self._loop_stop_name}")
        self.get_logger().info(f"vlm_url={self._vlm_url}")

    def destroy_node(self) -> bool:
        self._signal_stop_and_join()
        return super().destroy_node()

    def _signal_stop_and_join(self) -> None:
        self._loop_stop_event.set()
        with self._loop_thread_lock:
            t = self._loop_thread
        if t is not None and t.is_alive():
            t.join(timeout=self._http_timeout + 10.0)

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        with self._frame_lock:
            self._latest_bgr = bgr

    def _on_loop_start(
        self, request: VlmLoopStart.Request, response: VlmLoopStart.Response
    ) -> VlmLoopStart.Response:
        prompt = (request.prompt or "").strip()
        out_file = (request.output_file or "").strip()
        if not prompt:
            response.success = False
            response.error = "Empty prompt"
            return response
        if not out_file:
            response.success = False
            response.error = "Empty output_file"
            return response

        interval = float(request.interval_sec)
        if interval <= 0.0:
            interval = self._default_interval

        path = Path(out_file).expanduser()
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
        except OSError as e:
            response.success = False
            response.error = f"Cannot create parent directory for output file: {e}"
            return response

        with self._loop_thread_lock:
            if self._loop_thread is not None and self._loop_thread.is_alive():
                response.success = False
                response.error = "Loop already running; call stop first"
                return response
            self._loop_stop_event.clear()
            self._loop_thread = threading.Thread(
                target=self._loop_worker,
                args=(prompt, path.resolve(), interval),
                daemon=True,
                name="vlm_loop",
            )
            self._loop_thread.start()

        response.success = True
        response.error = ""
        self.get_logger().info(
            f"VLM loop started: interval={interval}s, output_file={path.resolve()}"
        )
        return response

    def _on_loop_stop(
        self, _request: VlmLoopStop.Request, response: VlmLoopStop.Response
    ) -> VlmLoopStop.Response:
        with self._loop_thread_lock:
            running = self._loop_thread is not None and self._loop_thread.is_alive()
        if not running:
            response.success = False
            response.error = "Loop is not running"
            return response

        self._loop_stop_event.set()
        with self._loop_thread_lock:
            t = self._loop_thread
        if t is not None:
            t.join(timeout=self._http_timeout + 10.0)

        response.success = True
        response.error = ""
        self.get_logger().info("VLM loop stopped")
        return response

    def _loop_worker(self, prompt: str, output_file: Path, interval: float) -> None:
        index = 0
        try:
            while not self._loop_stop_event.is_set():
                index += 1
                text, err = self._infer_one(prompt)
                if err:
                    body = f"[error] {err}"
                else:
                    body = (text or "").rstrip()

                stamp = time.strftime("%Y-%m-%d %H:%M:%S")
                block = f"\n--- {stamp} --- [{index}]\n{body}\n"
                try:
                    with open(output_file, "a", encoding="utf-8") as f:
                        f.write(block)
                except OSError as e:
                    self.get_logger().error(f"Failed to append to {output_file}: {e}")

                if self._loop_stop_event.wait(timeout=interval):
                    break
        finally:
            with self._loop_thread_lock:
                self._loop_thread = None
            self.get_logger().info("VLM loop worker exited")

    def _infer_one(self, prompt: str) -> Tuple[str, str]:
        """Returns (answer, error). If error is non-empty, answer is ignored."""
        with self._frame_lock:
            frame = None if self._latest_bgr is None else self._latest_bgr.copy()

        if frame is None:
            return "", "No image received yet"

        ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            return "", "Failed to encode image"

        data_url = "data:image/jpeg;base64," + base64.b64encode(buf.tobytes()).decode("utf-8")

        payload = {
            "messages": [
                {"role": "system", "content": self._system_prompt},
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
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
            return answer, ""
        except Exception as e:
            return "", str(e)


def main() -> None:
    rclpy.init()
    node = VlmServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
