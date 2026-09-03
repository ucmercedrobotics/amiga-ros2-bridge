#!/usr/bin/env python3
"""How an agent asks what the robot can see.

    triage  --/vlm/ask-->  vlm_server (amiga_vlm_bridge)  --> a vision model
            <--------  a sentence describing the frame  ---------

The agent calls one service and gets text back. Everything between -- holding
the latest frame, encoding it, talking to the model -- is the ROS node's
problem, not the agent's.

Never load-bearing. Every failure returns ``None`` and the caller renders its
prompt without the section: the mission planner waits ``ROUTE_TIMEOUT_SEC`` for
a routing verdict and then replans anyway, so a camera that is slow, down or
absent must cost a couple of seconds and get out of the way.
"""

import time
from typing import Optional

from amiga_vlm_interfaces.srv import VlmAsk

#: Seconds to wait for an answer. Well inside the budget the planner allows for
#: a routing verdict, which also has a reasoning-model call to make.
DEFAULT_TIMEOUT_SEC = 8.0

#: The service ``vlm_server`` serves. Absolute, like every other name in this
#: package; ``robot_agents.launch.py`` remaps it relative for a fleet.
DEFAULT_SERVICE = "/vlm/ask"

#: How long to wait for the server to appear before deciding it is not running.
DISCOVERY_TIMEOUT_SEC = 0.5

#: Cap on what reaches the prompt, which already carries 60 log lines and a
#: mission XML. Cut rather than refused: a clipped description is still evidence.
MAX_ANSWER_CHARS = 600


class VlmClient:
    """Asks the VLM about the current camera frame. Never raises."""

    def __init__(
        self,
        node,
        service_name: str = DEFAULT_SERVICE,
        timeout_sec: float = DEFAULT_TIMEOUT_SEC,
        callback_group=None,
    ):
        self._node = node
        self._timeout_sec = float(timeout_sec)
        self._client = node.create_client(
            VlmAsk, service_name, callback_group=callback_group
        )
        self.service_name = service_name

    def ask(self, question: str) -> Optional[str]:
        """One question about the latest frame. Returns the answer, or None.

        None covers every way this fails -- no server, no frame yet, a timeout,
        an error at the far end -- because the caller treats them all the same:
        leave the section out. The reason is logged either way.

        Blocks for up to ``timeout_sec``, and spinning is the caller's
        executor's job, so this must run on a background thread or in a
        reentrant callback group on a MultiThreadedExecutor. Triage does both.
        """
        if not self._client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT_SEC):
            self._node.get_logger().debug(
                f"no VLM on {self.service_name} — routing on logs alone"
            )
            return None

        request = VlmAsk.Request()
        request.question = question

        future = self._client.call_async(request)
        deadline = time.monotonic() + self._timeout_sec
        while not future.done():
            if time.monotonic() >= deadline:
                future.cancel()
                self._node.get_logger().warn(
                    f"VLM did not answer within {self._timeout_sec}s — "
                    "routing on logs alone"
                )
                return None
            time.sleep(0.02)

        response = future.result()
        if response is None:
            self._node.get_logger().warn("VLM call completed with no response")
            return None
        if not response.success:
            # "No image received yet" arrives here. Reported as an absence
            # rather than passed on as text: the model must never read a
            # diagnostic string as something the camera saw.
            self._node.get_logger().info(
                f"VLM declined: {response.error or 'no reason given'}"
            )
            return None

        answer = (response.answer or "").strip()
        if not answer:
            return None
        return answer[:MAX_ANSWER_CHARS]


#: The one question put to the camera. It asks for a description and nothing
#: else: judgements ("is anything in the way", "is the row passable") belong to
#: the reasoning model, and geometry to the robot's own sensors.
DESCRIBE_QUESTION = (
    "Describe what you see.\n\n"
    "Name the things in view and what each one is — people, animals, vehicles, "
    "machinery, trees, posts, buildings, ground, sky. Say what a thing is, not "
    "what it means.\n\n"
    "The robot's own arm — white, jointed, often large in the foreground — may "
    "be in view. you do not need to name it or describe it.\n\n"
    "If the image is blank, black, uniformly coloured, heavily blurred or "
    "washed out, say that instead of describing content.\n\n"
    "Report only what is visible. Do not estimate distances or bearings — the "
    "robot's own sensors have those. Do not say whether anything is in the "
    "way, whether a path is clear, whether the image is usable, or what the "
    "robot should do. Something else decides all of that from what you report."
)


DESCRIBE_FRAME_SENTENCE = "Describe the frame itself as well as what is in it."


def describe_question(include_frame: bool = False) -> str:
    """The question to put to the camera.

    ``include_frame`` adds ``DESCRIBE_FRAME_SENTENCE`` after the list of things
    to name, which is where it was measured -- the list stays, so a person is
    still called a person.
    """
    if not include_frame:
        return DESCRIBE_QUESTION
    marker = "what it means.\n\n"
    at = DESCRIBE_QUESTION.index(marker) + len(marker)
    return (
        DESCRIBE_QUESTION[:at]
        + DESCRIBE_FRAME_SENTENCE
        + "\n\n"
        + DESCRIBE_QUESTION[at:]
    )
