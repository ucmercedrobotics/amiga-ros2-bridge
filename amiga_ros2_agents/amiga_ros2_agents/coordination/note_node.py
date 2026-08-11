"""
note_node.py

The second of the two reasoning points: `interpret_note`.

`triage_node` reasons about *this* robot's fault, from evidence this robot
gathered, and decides what happens to work it cannot finish. This node reasons
about *another* robot's sentence, and decides what that sentence means for a bid
we were about to make. They are separate agents because they are separate
judgements from separate evidence -- one is diagnosis, the other is reading an
offer -- and a single prompt asked to do both would be a prompt whose context is
half irrelevant on every call.

    coordinator --/coordination/interpret_note--> here --> keep|revise|withdraw

Nothing is subscribed. Everything this decision needs arrives in the request:
the note, the offer it annotates, and what our own fitness already concluded.
That is not an accident of scope -- a note is *about* a specific announcement,
and context gathered from anywhere else would be context about a different
moment.

Why the answer is three words
-----------------------------

This is the only place in the system where text written by another robot
reaches a decision. The radio's `src` is self-asserted and unauthenticated, so
the sentence in front of this model could have been written by anyone in range,
including someone hostile. The prompt does not defend against that and is not
expected to. What defends against it is that the response is closed at three
values, all of which only adjust a bid this robot was already going to make. A
note cannot start an auction, take on work, drop work, or move the robot -- so
the worst outcome from the worst possible sentence is one bad bid, or silence.

The coordinator's client refuses anything outside those three, and this node
refuses it first, before it goes on the wire.

Timing is the other constraint. Reading a note stretches the bid backoff tenfold
(20 s at the coordinator's defaults) to make room for this call, and its client
gives up at 15 s. An answer after that is not late, it is useless: the bid has
gone. So the prompt asks for a short answer and this node does not retry.

Prompts live in prompts/note/.
"""

import json
from threading import Lock
from typing import Dict

from amiga_interfaces.srv import InterpretNote
from amiga_ros2_comms.codec import COST_MAX, Target, TargetKind
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from ..mission import mission_tasks
from ..runtime import llm, prompts, spin
from ..runtime.status import StatusPublisher

#: The closed set, restated here as well as in the coordinator's schema.py for
#: the same reason VALID_ACTIONS is: an answer outside it is only ever going to
#: be discarded downstream, and by then the note has cost a model call and the
#: auction has spent its deliberative window waiting for it.
VALID_REVISIONS = ("keep", "revise", "withdraw")


class NoteNode(Node):
    """Serves `coordination/interpret_note`."""

    def __init__(self):
        super().__init__("note_agent")
        self._lock = Lock()
        self._status = {
            "model": llm.MODEL,
            "notes_read": 0,
            "refused": 0,
            "last_revision": None,
            "last_error": None,
        }

        self.system_prompt = prompts.render(
            "note/system.j2",
            valid_revisions=VALID_REVISIONS,
            cost_max=COST_MAX,
        )

        # A model call takes seconds and the service is the only thing this
        # node does, but a reentrant group plus the multithreaded executor in
        # main() means two notes about two different tasks -- which is what a
        # fleet mid-auction produces -- are read concurrently rather than
        # queued behind each other past the bid deadline.
        self.create_service(
            InterpretNote,
            "/coordination/interpret_note",
            self._on_interpret,
            callback_group=ReentrantCallbackGroup(),
        )

        self.status = StatusPublisher(self)
        self.status.publish(self.get_status())
        self.get_logger().info(
            f"NoteNode started — model={llm.MODEL}, serving "
            f"/coordination/interpret_note"
        )

    def get_status(self) -> Dict:
        with self._lock:
            return dict(self._status)

    # ------------------------------------------------------------------

    def _on_interpret(self, request, response):
        with self._lock:
            self._status["notes_read"] += 1

        try:
            user_prompt = prompts.render(
                "note/user.j2",
                note_text=request.note_text or "(empty)",
                announcer_id=int(request.announcer_id),
                task_id=int(request.task_id),
                # Element names, not a mask -- the same words the mission XML
                # is written in, so the model is not asked to decode a number.
                required_actions=", ".join(
                    mission_tasks.capability_names(int(request.required_capabilities))
                )
                or "(not stated)",
                where=self._where(request),
                priority=int(request.priority),
                bid_known=not bool(request.our_bid_unknown),
                our_cost=int(request.our_cost),
                our_eta_s=int(request.our_eta_s),
                our_feasible=bool(request.our_feasible),
                cost_max=COST_MAX,
            )
            reply = llm.complete(self.system_prompt, user_prompt)
            decision = self._parse_revision(reply)
        except Exception as exc:  # noqa: BLE001 - a model, a parser, a timeout
            self.get_logger().error(f"note interpretation failed: {exc}")
            with self._lock:
                self._status["refused"] += 1
                self._status["last_error"] = str(exc)
            response.ok = False
            response.error = str(exc)
            response.revision = ""
            self.status.publish(self.get_status())
            return response

        response.ok = True
        response.error = ""
        response.revision = decision["revision"]
        response.cost_delta = decision["cost_delta"]
        response.reason = decision["reason"]
        response.rationale = decision["rationale"]
        response.model = llm.MODEL

        with self._lock:
            self._status["last_revision"] = decision["revision"]
            self._status["last_error"] = None
        self.get_logger().info(
            f"note on task {int(request.task_id)} from robot "
            f"{int(request.announcer_id)}: {decision['revision']}"
            + (
                f" ({decision['cost_delta']:+d})"
                if decision["revision"] == "revise"
                else ""
            )
            + f" — {decision['reason'][:120]}"
        )
        self.status.publish(self.get_status())
        return response

    # ------------------------------------------------------------------

    @staticmethod
    def _where(request) -> str:
        """The task's place, in words, or that the offer has not arrived yet.

        `target_kind` NONE is genuinely ambiguous here: it is both "work that
        happens wherever the robot is" and the zero a request carries when the
        note beat its announcement. `our_bid_unknown` is what distinguishes
        them, and the user prompt says which one this is.
        """
        try:
            return str(
                Target(
                    kind=TargetKind(int(request.target_kind)),
                    a=int(request.target_a),
                    b=int(request.target_b),
                )
            )
        except ValueError:
            return "(somewhere this robot cannot name)"

    def _parse_revision(self, text: str) -> Dict:
        """One JSON object in, a checked revision out. Raises on anything else.

        Deliberately the same shape as triage's parser, including refusing a
        list: a reply offering two revisions is a reply where taking the first
        would be acting on a decision nobody made.
        """
        start, end = text.find("{"), text.rfind("}")
        if start == -1 or end <= start:
            raise ValueError(f"no JSON object in the reply: {text[:200]!r}")
        bracket = text.find("[")
        if bracket != -1 and bracket < start:
            raise ValueError("expected one revision object, got a list of them")
        decision = json.loads(text[start : end + 1])
        if not isinstance(decision, dict):
            raise ValueError(f"expected a JSON object, got {type(decision).__name__}")

        revision = str(decision.get("revision", "")).strip().lower()
        if revision not in VALID_REVISIONS:
            raise ValueError(
                f"revision {revision!r} is not one of {', '.join(VALID_REVISIONS)}"
            )

        delta = 0
        if revision == "revise":
            raw = decision.get("cost_delta")
            try:
                delta = int(raw)
            except (TypeError, ValueError):
                raise ValueError(
                    f"revise needs a numeric cost_delta; got {raw!r}"
                ) from None
            if delta == 0:
                # "Revise by nothing" is keep, said in a way the coordinator
                # would log as a revision. Refusing makes the model say which
                # it meant rather than leaving the counters to lie about it.
                raise ValueError("revise with cost_delta 0 is keep; say keep")
            delta = max(-COST_MAX, min(delta, COST_MAX))

        return {
            "revision": revision,
            "cost_delta": delta,
            "reason": str(decision.get("reason", "")).strip(),
            "rationale": str(decision.get("rationale", "")).strip(),
        }


def main():
    # Multithreaded for the reason in __init__: concurrent notes, each with a
    # deadline shorter than a model call is guaranteed to be.
    spin.run(NoteNode, multithreaded=True)


if __name__ == "__main__":
    main()
