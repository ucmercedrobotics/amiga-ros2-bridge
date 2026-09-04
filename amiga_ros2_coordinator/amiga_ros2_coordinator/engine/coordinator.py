#!/usr/bin/env python3
"""Contract net, both roles, deterministic. The engine, with no ROS in it.

One instance per robot. It is simultaneously a potential **owner** -- it has
tasks it may need to shed -- and a potential **bidder** -- it may take on a
peer's. Both roles run here at once, because a robot that could only do one of
them could not participate in a fleet where every robot does both.

    owner   infeasible -> interpret -> ANNOUNCE -> collect -> arbitrate
                       -> GRANT -> *delivered* -> transferred -> replan
    bidder  ANNOUNCE -> capable? -> ask nav/mission -> fitness -> backoff
                     -> (suppressed?) -> BID -> GRANT for us -> absorb -> replan

The rule that shapes everything else is **unassigned-until-ACKed**. A task is
ours from the moment the mission node hands it to us until the reliability
layer confirms that another robot acknowledged the GRANT -- announced is still
ours, granted-but-unconfirmed is still ours. Sending a GRANT is not a transfer;
a delivery report is. The alternative is a window in which a task belongs to
nobody, and a task belonging to nobody is a row that never gets sprayed and
nobody notices.

Two things this file deliberately does not do:

**It does not reason.** The two open-ended questions -- what to do about an
anomaly, and whether the re-planned mission still verifies -- are calls to
injected interfaces that are stubbed today (see reasoning.py). Everything here
is a decision procedure that can be predicted from its inputs, which is what
makes nine acceptance tests a meaningful claim about it.

**It does not interrupt.** Coordination events arrive whenever the radio says
so, and a robot mid-way through an arm move must not be stopped by one. This
layer raises a preemption flag and the behaviour tree yields at a tick point it
has chosen to be safe. There is no code path here that stops an action.

Conventions inherited from the layer below, for the same reasons:

**The clock is injected.** ``clock()`` returns monotonic seconds. A test drives
whole auctions, backoff windows and peer timeouts in microseconds
with nothing sleeping and no flakiness.

**Effects are callables.** Transport is the reliability port; navigation and
the mission stack are the protocols in interfaces.py. Nothing here knows
whether it is talking to a radio or to a dict.

Threading: public methods take a re-entrant lock, and the two callbacks that
arrive from the reliability layer's thread (an inbound message, a delivery
outcome) take the same one. The reliability layer invokes both with its own
lock released, so there is no lock-order inversion between the two.
"""

import random
import threading
import time
from dataclasses import dataclass, field, replace
from typing import Callable, Dict, List, Optional

from amiga_ros2_comms.codec import (
    COST_MAX,
    TASK_NONE,
    Bid,
    Grant,
    Heartbeat,
    Message,
    ReasonCode,
    TaskAnnounce,
    Target,
    cap_mask,
    has_capabilities,
    target_fields,
    target_of,
)
from amiga_ros2_comms.reliability import CompletedNote, NoteTooLong, Outcome

from .auction import Auction, ReceivedBid
from .bidding import (
    DEFAULT_JITTER_FRACTION,
    DEFAULT_MAX_BACKOFF_SEC,
    PendingBid,
    default_fitness,
    make_backoff,
    quantized_eta,
)
from ..ports.interfaces import (
    MissionInterface,
    NavInterface,
    NullPreemption,
    PreemptionSignal,
)
from ..vocabulary.model import Fitness, MissionDelta, Task, TaskState
from ..ports.reasoning import (
    AnomalyInterpreter,
    IgnoreNotes,
    MissionReplanner,
    NoteInterpreter,
    ReplanResult,
)
from .registry import DEFAULT_PEER_TIMEOUT_SEC, PeerRegistry
from ..vocabulary.schema import (
    AddTask,
    AnomalyContext,
    DropTask,
    KeepBid,
    LocalDisposition,
    NoteContext,
    ReDelegate,
    ReviseBid,
    WithdrawBid,
    validate_action,
    validate_revision,
)


@dataclass(frozen=True)
class CoordinatorParams:
    """Everything tunable, in one object.

    The relationship to get right is ``bid_max_backoff_sec`` against
    ``announce_window_sec``. The backoff exists to make the best bid arrive
    first into a quiet channel; if the backoff can outlast the window, the best
    bid arrives after the auction closed and the mechanism achieves the exact
    opposite of its purpose. That is checked below rather than left to be
    discovered as "the good robot never wins".

    It is a *fleet-wide* relationship -- our window is checked against our
    backoff, but the bidder that matters is on another robot. The check is
    still worth having: these are deployed from one config, and a fleet where
    the two disagree is a fleet where nobody's auction works.
    """

    #: How long an announcement collects bids before it is arbitrated.
    announce_window_sec: float = 5.0
    #: Seconds between re-broadcasts of an announcement while its window is
    #: open. Announcements are best-effort broadcasts, so repetition is the
    #: only way one reaches a robot that was transmitting when the first went
    #: out. 0 disables it.
    announce_repeat_sec: float = 2.0
    #: Longest a bidder sits on a bid. See the class docstring.
    bid_max_backoff_sec: float = DEFAULT_MAX_BACKOFF_SEC
    bid_jitter_fraction: float = DEFAULT_JITTER_FRACTION

    # -- the deliberative clock -------------------------------------------
    #
    # An auction with a note attached runs on these instead. Reading a note is
    # a language-model call taking seconds, and the fast values above are
    # sub-second for exactly the bidder whose mind the note is most worth
    # changing -- backoff is linear in cost, so the best-fitting bidder waits
    # least. There is no arrangement of a 2 s backoff and a 5 s window that
    # gets an interpretation in before the bid goes out.
    #
    # So a note-bearing auction is a slower auction, deliberately and
    # measurably. That is the trade the whole feature makes: allocation latency
    # for allocation quality. Nothing here paces the radio -- the packets go out
    # as fast as ever, we just wait longer before deciding.
    #
    # Multipliers rather than absolute seconds, because there is only one
    # auction clock here and this is a factor applied to it. A fleet that runs
    # short auctions gets a proportionally short deliberative auction, and
    # every relationship the fast values already satisfy -- backoff inside the
    # window, bid memory outlasting the window -- keeps holding when both sides
    # scale together. Two absolute settings would let a deployment tune one and
    # silently break the other.
    #: Window for an auction whose announcement carries a note, as a multiple
    #: of ``announce_window_sec``. 9x the 5 s default is 45 s -- room for a
    #: model call plus the backoff that follows it.
    note_window_multiplier: float = 9.0
    #: Longest a bidder sits on a bid it is having a note interpreted for, as a
    #: multiple of ``bid_max_backoff_sec``. 18x the 2 s default is 36 s. Sized
    #: from measurement, not taste: a 120B note interpretation was observed
    #: taking 21-22 s against three robots sharing one endpoint, so the 10x
    #: (20 s) this used to be put every honest answer past its own deadline.
    note_backoff_multiplier: float = 18.0

    @property
    def note_announce_window_sec(self) -> float:
        """The deliberative window, in seconds."""
        return self.announce_window_sec * self.note_window_multiplier

    @property
    def note_bid_max_backoff_sec(self) -> float:
        """The deliberative backoff ceiling, in seconds."""
        return self.bid_max_backoff_sec * self.note_backoff_multiplier

    #: How long we remember having bid on a task, so a GRANT that arrives after
    #: our bid can still be matched to it. Must outlast the announcer's window
    #: plus its whole GRANT retransmit campaign.
    bid_memory_sec: float = 60.0
    #: Attempts at delegating one task before it goes to local handling. An
    #: attempt is one full announce-and-grant cycle; a GRANT that fails to
    #: deliver consumes one.
    max_delegation_attempts: int = 2
    #: Shortest interval between re-announcements of the *same* task, so a
    #: mission node reporting the same failure in a loop cannot turn into an
    #: announce storm.
    redelegation_cooldown_sec: float = 30.0
    #: Seconds without a HEARTBEAT before a peer is presumed gone.
    peer_timeout_sec: float = DEFAULT_PEER_TIMEOUT_SEC
    #: How often we emit our own HEARTBEAT. 0 disables emission.
    heartbeat_period_sec: float = 10.0
    #: How long a settled task (transferred, relinquished) is kept for
    #: introspection before being forgotten.
    settled_retention_sec: float = 300.0
    #: Auctions we will run at once. Beyond it, re-delegation is refused and
    #: the task goes to local handling -- a robot shedding everything at once
    #: is a robot whose problem is not delegable.
    max_open_auctions: int = 8

    def __post_init__(self):
        if self.announce_window_sec <= 0:
            raise ValueError("announce_window_sec must be positive")
        if self.bid_max_backoff_sec < 0:
            raise ValueError("bid_max_backoff_sec must be >= 0")
        if not 0.0 <= self.bid_jitter_fraction < 1.0:
            raise ValueError("bid_jitter_fraction must be in [0.0, 1.0)")
        worst_backoff = self.bid_max_backoff_sec * (1.0 + self.bid_jitter_fraction)
        if worst_backoff >= self.announce_window_sec:
            raise ValueError(
                f"bid_max_backoff_sec={self.bid_max_backoff_sec} (up to "
                f"{worst_backoff:.2f}s with jitter) is not shorter than "
                f"announce_window_sec={self.announce_window_sec}. The worst-fitting "
                f"bidder would transmit after the auction closed, so fitness "
                f"ordering would decide nothing."
            )
        # The same relationship again, on the deliberative clock. It does not
        # follow from the fast one: the two multipliers are independent, and
        # scaling the backoff faster than the window is exactly how a
        # deliberative auction would close before its bids went out.
        if self.note_window_multiplier < 1.0:
            raise ValueError(
                "note_window_multiplier must be >= 1.0; an auction that has to "
                "wait for an interpretation cannot be shorter than one that "
                "does not"
            )
        if self.note_backoff_multiplier < 1.0:
            raise ValueError("note_backoff_multiplier must be >= 1.0")
        worst_note_backoff = self.note_bid_max_backoff_sec * (
            1.0 + self.bid_jitter_fraction
        )
        if worst_note_backoff >= self.note_announce_window_sec:
            raise ValueError(
                f"note_backoff_multiplier={self.note_backoff_multiplier} gives a "
                f"deliberative backoff of up to {worst_note_backoff:.2f}s with "
                f"jitter, which is not shorter than the deliberative window of "
                f"{self.note_announce_window_sec:.2f}s "
                f"(note_window_multiplier={self.note_window_multiplier}). The "
                f"worst-fitting bidder would transmit after the auction closed."
            )
        if self.max_delegation_attempts < 1:
            raise ValueError("max_delegation_attempts must be >= 1")
        if self.bid_memory_sec <= self.announce_window_sec:
            raise ValueError(
                "bid_memory_sec must outlast announce_window_sec, or a GRANT "
                "can arrive after we have forgotten bidding"
            )


@dataclass
class OwnedTask:
    """A task this robot is responsible for, and what is happening to it.

    Responsible for, not necessarily executing: a task under auction and a task
    whose GRANT is unconfirmed are both still ours. That is the
    unassigned-until-ACKed rule, made into a field you can print.
    """

    task: Task
    state: TaskState = TaskState.OURS
    auction: Optional[Auction] = None
    #: What to do if delegation fails outright. From the ReDelegate action.
    fallback: LocalDisposition = LocalDisposition.HOLD
    reason_code: int = ReasonCode.UNSPECIFIED
    #: Free text to broadcast alongside the announcement, from the ReDelegate
    #: action. Empty means an ordinary fast auction; anything else makes this a
    #: deliberative one, on the longer clock.
    note: str = ""
    #: Full announce-and-grant cycles spent on this task.
    delegation_attempts: int = 0
    #: Set once a GRANT is in flight; cleared if it fails to deliver.
    granted_to: Optional[int] = None
    #: Bidders whose GRANT could not be delivered. Not offered it again.
    excluded_bidders: "set[int]" = field(default_factory=set)
    #: Clock reading of the last announcement, for the cooldown.
    last_announced_at: float = float("-inf")
    #: When the task reached a terminal state, for pruning.
    settled_at: Optional[float] = None
    #: Whether the task is currently part of our own mission plan.
    #:
    #: The invariant: **the mission holds a task iff we still intend to execute
    #: it ourselves.** Announcing means we do not, so a task leaves at announce
    #: time rather than at transfer time -- otherwise the announcer sits on work
    #: it has already said it cannot do for the length of an auction, which on
    #: the deliberative clock is most of a minute.
    #:
    #: A field rather than something inferred from ``state``, because it is what
    #: keeps every replan a real change: an auction can fail and hand the task
    #: back, and the verifier must be told that exactly once.
    in_mission: bool = True

    @property
    def ours(self) -> bool:
        """The unassigned-until-ACKed predicate, in one place."""
        return self.state in (TaskState.OURS, TaskState.ANNOUNCED, TaskState.GRANTED)


class CoordinatorSession:
    """The contract-net state machine. Both roles, no ROS, no reasoning."""

    def __init__(
        self,
        node_id: int,
        reliability,
        nav: NavInterface,
        mission: MissionInterface,
        interpreter: AnomalyInterpreter,
        replanner: MissionReplanner,
        capabilities=(),
        preemption: Optional[PreemptionSignal] = None,
        params: Optional[CoordinatorParams] = None,
        clock: Optional[Callable[[], float]] = None,
        fitness_fn: Optional[Callable[..., Fitness]] = None,
        rng: Optional[random.Random] = None,
        on_event: Optional[Callable[[str, str], None]] = None,
        note_interpreter: Optional[NoteInterpreter] = None,
    ):
        if not 0 < int(node_id) <= 255:
            raise ValueError(f"node_id must be 1..255, got {node_id}")

        self.node_id = int(node_id)
        self.params = params or CoordinatorParams()
        self._reliability = reliability
        self._nav = nav
        self._mission = mission
        self._interpreter = interpreter
        self._replanner = replanner
        self._preemption = preemption or NullPreemption()
        self._clock = clock or time.monotonic
        self._fitness_fn = fitness_fn or default_fitness
        self._on_event = on_event
        self._lock = threading.RLock()

        self.cap_mask = cap_mask(*capabilities) if capabilities else 0

        self._interpret_note = note_interpreter or IgnoreNotes()

        self._backoff = make_backoff(
            self.params.bid_max_backoff_sec,
            self.params.bid_jitter_fraction,
            rng,
        )
        # The same function on the deliberative clock. Two bound functions
        # rather than one that takes a window, so every call site has already
        # decided which auction it is in and cannot forget to say.
        self._note_backoff = make_backoff(
            self.params.note_bid_max_backoff_sec,
            self.params.bid_jitter_fraction,
            rng,
        )

        self.registry = PeerRegistry(
            timeout_sec=self.params.peer_timeout_sec,
            clock=self._clock,
            on_change=self._on_peer_change,
        )

        self._tasks: Dict[int, OwnedTask] = {}
        self._bids: Dict[int, PendingBid] = {}
        self._next_heartbeat_at = 0.0
        # Notes waiting to be interpreted. Filled under the lock by whichever
        # of the two arrival orders happened; drained off it by the node, which
        # makes the model call and comes back through revise_bid. The same
        # two-door shape as anomaly_context / report_infeasible(action=...),
        # and for the same reason -- a model call must not run on this lock.
        self._note_requests: List[NoteContext] = []

        self._counters = {
            "anomalies": 0,
            "interpret_failed": 0,
            "announced": 0,
            "re_announced": 0,
            "delegation_refused": 0,
            "auctions_won": 0,
            "auctions_no_bid": 0,
            "grants_sent": 0,
            "grants_delivered": 0,
            "grants_failed": 0,
            "tasks_transferred": 0,
            "tasks_absorbed": 0,
            "tasks_dropped": 0,
            "tasks_held": 0,
            "escalated_to_human": 0,
            "bids_considered": 0,
            "bids_incapable": 0,
            "bids_sent": 0,
            "bids_infeasible": 0,
            "bids_suppressed": 0,
            "bids_lost": 0,
            "grants_unmatched": 0,
            "replans": 0,
            "replans_rejected": 0,
            "heartbeats_sent": 0,
            "preemptions_requested": 0,
            "rx_ignored": 0,
            # Notes. notes_sent/notes_received are traffic; the rest are the
            # question the experiment is actually asking -- how often does text
            # reach a bidder in time to change anything, and what does it change.
            "notes_sent": 0,
            "notes_received": 0,
            "notes_before_announce": 0,
            "notes_after_announce": 0,
            "notes_orphaned": 0,
            "notes_too_late": 0,
            "note_interpret_failed": 0,
            "bids_kept_after_note": 0,
            "bids_revised_by_note": 0,
            "bids_withdrawn_by_note": 0,
        }

    # ==================================================================
    # Inbound from the reliability layer
    # ==================================================================

    def on_message(self, msg: Message) -> None:
        """One deduplicated inbound message. Never raises.

        Installed as the reliability layer's ``on_deliver``. It is the only
        thing between decoded radio traffic and the state machine, so a peer
        sending something we did not expect must not be able to stop
        coordination -- the same argument that makes ``on_frame`` never raise
        one layer down.
        """
        try:
            with self._lock:
                now = self._clock()
                if int(getattr(msg, "src", 0)) == self.node_id:
                    # Self-echo. Reliability drops these, but a repeater plus a
                    # misconfiguration is exactly the situation where being
                    # defensive twice costs nothing.
                    self._counters["rx_ignored"] += 1
                    return
                if isinstance(msg, Heartbeat):
                    self.registry.observe(msg, now)
                elif isinstance(msg, TaskAnnounce):
                    self._on_announce(msg, now)
                elif isinstance(msg, Bid):
                    self._on_bid(msg, now)
                elif isinstance(msg, Grant):
                    self._on_grant(msg, now)
                else:
                    self._counters["rx_ignored"] += 1
        except Exception as exc:  # noqa: BLE001 - never let the radio kill us
            self._event("warn", f"handling {type(msg).__name__} raised: {exc}")

    # ==================================================================
    # Owner role
    # ==================================================================

    def own(self, task: Task) -> OwnedTask:
        """Register ``task`` as this robot's responsibility.

        Called by the mission node when it takes work on. Idempotent, so a
        mission node that re-announces its own plan does not reset an auction
        already in flight.
        """
        with self._lock:
            existing = self._tasks.get(task.task_id)
            if existing is not None and existing.ours:
                return existing
            record = OwnedTask(task=task)
            self._tasks[task.task_id] = record
            return record

    def report_infeasible(
        self,
        task: Optional[Task],
        detail: str = "",
        reason_code: int = ReasonCode.TASK_FAILED,
        action: Optional[object] = None,
    ) -> Optional[object]:
        """The mission node cannot complete ``task``. Decide what to do.

        This is the infeasibility signal the whole owner role hangs off, and
        the one place ``interpret_anomaly`` is called. Returns the action that
        was chosen, or None if no interpretation was available -- in which case
        the task stays exactly as it was, which is the safe direction to fail
        in: an uninterpretable anomaly leaves work owned by a robot that knows
        it is stuck, rather than owned by nobody.

        ``action`` is for a caller that has *already* obtained an
        interpretation and only needs it executed. That exists because the real
        interpreter is a service call to a language model that takes seconds,
        and this method holds the lock that ``tick`` and ``on_message`` need:
        running it inline would stop heartbeats, auctions and bids for the
        duration of a model call. The node therefore resolves the decision off
        this lock and hands the finished action back here. It is validated
        exactly as an interpreter's answer would be -- the closed union is the
        guarantee, and it does not get to be skipped by taking a different door.
        """
        with self._lock:
            now = self._clock()
            self._counters["anomalies"] += 1

            if task is not None and task.task_id not in self._tasks:
                # A failure report about work we were never told we had. Take
                # ownership rather than refuse: the mission node is the
                # authority on what it is executing, and refusing would leave
                # the task with nobody looking after it.
                self._tasks[task.task_id] = OwnedTask(task=task)

            if action is None:
                context = AnomalyContext(
                    task=task,
                    detail=detail,
                    reason_code=reason_code,
                    peers=self.registry.peers,
                    battery=self._safe(self._mission.battery_percent, 0),
                    location=self._safe(self._nav.current_location, None),
                    at=now,
                )
                try:
                    action = self._interpreter.interpret_anomaly(context)
                except Exception as exc:  # noqa: BLE001 - a model, a parser, anything
                    self._counters["interpret_failed"] += 1
                    self._event(
                        "warn", f"interpret_anomaly failed, task unchanged: {exc}"
                    )
                    return None

            try:
                action = validate_action(action)
            except Exception as exc:  # noqa: BLE001 - a parser that half-succeeded
                self._counters["interpret_failed"] += 1
                self._event("warn", f"interpretation refused, task unchanged: {exc}")
                return None

            self._apply(action, now)
            return action

    def anomaly_context(
        self,
        task: Optional[Task],
        detail: str = "",
        reason_code: int = ReasonCode.TASK_FAILED,
    ) -> AnomalyContext:
        """The snapshot an out-of-band interpreter should reason over.

        Split out so the node can assemble it under the lock -- cheaply -- and
        then release the lock for the seconds a model call takes. A snapshot
        rather than a live handle for the reason given on AnomalyContext: a
        model reasoning over state that moves underneath it produces decisions
        about a robot that no longer exists.
        """
        with self._lock:
            return AnomalyContext(
                task=task,
                detail=detail,
                reason_code=reason_code,
                peers=self.registry.peers,
                battery=self._safe(self._mission.battery_percent, 0),
                location=self._safe(self._nav.current_location, None),
                at=self._clock(),
            )

    def _apply(self, action, now: float) -> None:
        """Execute one typed action from the anomaly interpretation."""
        if isinstance(action, ReDelegate):
            self._begin_delegation(action, now)
        elif isinstance(action, AddTask):
            self._take_on(action.task, now, cause="add_task")
        elif isinstance(action, DropTask):
            record = self._tasks.get(action.task.task_id) or OwnedTask(task=action.task)
            self._tasks[action.task.task_id] = record
            self._dispose_locally(record, action.disposition, now)

    # ------------------------------------------------------------------
    # Announce
    # ------------------------------------------------------------------

    def _begin_delegation(self, action: ReDelegate, now: float) -> None:
        record = self._tasks.get(action.task.task_id)
        if record is None:
            record = OwnedTask(task=action.task)
            self._tasks[action.task.task_id] = record
        record.fallback = action.fallback
        record.reason_code = action.reason_code
        record.note = action.note

        if record.state in (TaskState.ANNOUNCED, TaskState.GRANTED):
            # Already in flight. Re-entering would abandon the bids we have
            # collected and restart the clock, which is how a task that is
            # nearly delegated never quite gets delegated.
            return

        if not record.task.delegable:
            # The task has no place another robot could be sent to: a bare
            # SampleLeaf, or a subtree whose only movement is relative to this
            # robot's own pose. Announcing it would put a task on the air that
            # every bidder would have to interpret as "wherever you happen to
            # be", and the winner would do the work in the wrong place.
            # Falling back is the right answer and not a failure of the
            # auction -- it is a fact about the work.
            self._counters["delegation_refused"] += 1
            self._event(
                "warn",
                f"task {record.task.task_id} has no place another robot could "
                f"be sent to ({record.task.location}); handling it locally",
            )
            self._dispose_locally(record, record.fallback, now)
            return

        if now - record.last_announced_at < self.params.redelegation_cooldown_sec:
            self._counters["delegation_refused"] += 1
            self._event(
                "warn",
                f"task {record.task.task_id} re-announced within the "
                f"{self.params.redelegation_cooldown_sec}s cooldown; handling locally",
            )
            self._dispose_locally(record, record.fallback, now)
            return

        open_auctions = sum(
            1 for r in self._tasks.values() if r.state == TaskState.ANNOUNCED
        )
        if open_auctions >= self.params.max_open_auctions:
            self._counters["delegation_refused"] += 1
            self._event(
                "warn",
                f"{open_auctions} auctions already open (max_open_auctions); "
                f"handling task {record.task.task_id} locally instead",
            )
            self._dispose_locally(record, record.fallback, now)
            return

        self._announce(record, now)

    def _announce(self, record: OwnedTask, now: float) -> None:
        """Open an auction for ``record`` and put the first ANNOUNCE on the air."""
        capable = self.registry.capable(record.task.required_capabilities)
        record.state = TaskState.ANNOUNCED
        record.delegation_attempts += 1
        record.last_announced_at = now

        # The note goes out *before* the announcement, always. A bidder fixes
        # its bid the instant an ANNOUNCE decodes, so text arriving afterwards
        # has missed the only synchronous moment there is -- it can still
        # revise a bid that has not been transmitted, but only this ordering
        # gets it there reliably. Sent first even though it is several packets
        # and the announcement is one.
        sent_note = self._emit_note(record)
        window = (
            self.params.note_announce_window_sec
            if sent_note
            else self.params.announce_window_sec
        )

        record.auction = Auction(
            task=record.task,
            expected_bidders=frozenset(
                peer.robot_id
                for peer in capable
                if peer.robot_id not in record.excluded_bidders
            ),
            opened_at=now,
            closes_at=now + window,
            next_announce_at=now + self.params.announce_repeat_sec,
        )
        self._emit_announce(record, now)
        self._counters["announced"] += 1
        self._event(
            "info",
            f"announced task {record.task.task_id} "
            f"({len(record.auction.expected_bidders)} capable peers), "
            f"window {window}s{' with a note' if sent_note else ''}",
        )

        # And now stop waiting for it. We reported this task infeasible, which
        # is why it is on the air at all, so continuing to hold it in our own
        # mission would have us sitting on work we already said we cannot do --
        # for a whole deliberative window, which is where this stops being
        # theoretical. The invariant is: the mission holds a task iff we still
        # intend to execute it ourselves. Announcing means we do not.
        #
        # The counterpart is in _dispose_locally: an auction that fails has to
        # put the task back, because by then it is ours again.
        self._request_yield(f"task {record.task.task_id} announced to the fleet")
        self._leave_mission(record, "announced to the fleet")

    def _emit_note(self, record: OwnedTask) -> bool:
        """Broadcast the task's note, if it has one. Returns whether it went.

        A note failing to go out is not an auction failing. The announcement
        carries the whole machine-readable requirement by itself, so the worst
        case here is the fast auction we would have had anyway -- which is why
        this catches rather than propagates, and why the window is chosen from
        the *result* of this call rather than from whether a note exists.
        """
        if not record.note:
            return False
        try:
            fragments = self._reliability.send_note(record.task.task_id, record.note)
        except NoteTooLong as exc:
            self._event(
                "warn",
                f"note for task {record.task.task_id} is too long to send "
                f"({exc}); announcing without it",
            )
            return False
        except Exception as exc:  # noqa: BLE001 - the radio, the codec, anything
            self._event(
                "warn",
                f"note for task {record.task.task_id} could not be sent: {exc}; "
                f"announcing without it",
            )
            return False
        self._counters["notes_sent"] += 1
        self._event(
            "info",
            f"note for task {record.task.task_id} sent as {fragments} fragment(s)",
        )
        return True

    def _emit_announce(self, record: OwnedTask, now: float) -> None:
        task = record.task
        self._reliability.send_broadcast(
            TaskAnnounce(
                src=0,
                seq=0,
                task_id=task.task_id,
                req_cap_mask=int(task.required_capabilities),
                **target_fields(task.location),
                priority=int(task.priority),
                reason_code=int(record.reason_code),
            )
        )
        if record.auction is not None:
            record.auction.announcements += 1
            record.auction.next_announce_at = (
                now + self.params.announce_repeat_sec
                if self.params.announce_repeat_sec > 0
                else float("inf")
            )

    # ------------------------------------------------------------------
    # Collect and arbitrate
    # ------------------------------------------------------------------

    def _on_bid(self, bid: Bid, now: float) -> None:
        """A BID arrived. It is either an answer to us, or a rival to ours."""
        # Owner side: an answer to an auction we are running.
        record = self._tasks.get(int(bid.task_id))
        if (
            record is not None
            and record.state == TaskState.ANNOUNCED
            and record.auction is not None
        ):
            if int(bid.src) in record.excluded_bidders:
                # A peer whose GRANT we already failed to deliver. Its bid is
                # an offer from a robot we cannot reach.
                self._counters["rx_ignored"] += 1
                return
            if record.auction.record(bid, now):
                self._maybe_close(record, now)
                return

        # Bidder side: a rival for a task we are still sitting on.
        pending = self._bids.get(int(bid.task_id))
        if pending is not None and pending.sent_at is None:
            self._consider_suppression(pending, bid, now)

    def _maybe_close(self, record: OwnedTask, now: float) -> None:
        auction = record.auction
        if auction is None or not auction.should_close(now):
            return
        winner = auction.winner()
        if winner is None:
            self._no_viable_bid(record, now)
        else:
            self._grant(record, winner, now)

    def _no_viable_bid(self, record: OwnedTask, now: float) -> None:
        """The window closed and nobody viable answered.

        Delegation is over -- not retried. Nobody bidding is a fact about the
        fleet, and re-announcing into the same fleet a second later asks the
        same robots the same question. The task goes to whatever local handling
        the interpretation named when it chose to delegate.
        """
        heard = len(record.auction) if record.auction else 0
        record.auction = None
        record.state = TaskState.OURS
        self._counters["auctions_no_bid"] += 1
        self._event(
            "warn",
            f"task {record.task.task_id}: no viable bid from {heard} answer(s); "
            f"handling locally ({record.fallback.value})",
        )
        self._dispose_locally(record, record.fallback, now)

    # ------------------------------------------------------------------
    # Grant, and unassigned-until-ACKed
    # ------------------------------------------------------------------

    def _grant(self, record: OwnedTask, winner: ReceivedBid, now: float) -> None:
        """Award the task, and do *not* consider it gone.

        The order here is load-bearing. The state moves to GRANTED before the
        send, because a mock -- or a real link that ACKs impossibly fast --
        can resolve the future inside ``send_reliable`` and run the outcome
        callback before this method returns. Recording the outcome against a
        task still marked ANNOUNCED would lose the transfer.
        """
        record.state = TaskState.GRANTED
        record.granted_to = winner.bidder_id
        self._counters["auctions_won"] += 1

        task_id = record.task.task_id
        try:
            future = self._reliability.send_reliable(
                winner.bidder_id,
                Grant(src=0, seq=0, task_id=task_id, winner_id=winner.bidder_id),
            )
        except Exception as exc:  # noqa: BLE001 - a refused send is not a transfer
            self._event("warn", f"GRANT for task {task_id} refused: {exc}")
            self._grant_failed(record, winner.bidder_id, now)
            return

        self._counters["grants_sent"] += 1
        self._event(
            "info",
            f"granted task {task_id} to robot {winner.bidder_id} "
            f"(cost={winner.cost}, eta={winner.eta_s}s); awaiting delivery",
        )
        future.add_done_callback(
            lambda done, tid=task_id, wid=winner.bidder_id: self._on_grant_outcome(
                tid, wid, done
            )
        )

    def _on_grant_outcome(self, task_id: int, winner_id: int, future) -> None:
        """Reliability has resolved the GRANT. *This* is where a task moves.

        Runs on the reliability layer's thread, with its lock released.
        """
        try:
            outcome = future.result()
        except Exception as exc:  # noqa: BLE001 - a cancelled or broken future
            self._event("warn", f"GRANT future for task {task_id} raised: {exc}")
            outcome = Outcome.FAILED

        with self._lock:
            now = self._clock()
            record = self._tasks.get(task_id)
            if record is None or record.state != TaskState.GRANTED:
                # The task moved on underneath us -- dropped by an operator, or
                # a second outcome for a send we already resolved.
                self._counters["rx_ignored"] += 1
                return
            if outcome is Outcome.DELIVERED:
                self._grant_delivered(record, winner_id, now)
            else:
                self._grant_failed(record, winner_id, now)

    def _grant_delivered(self, record: OwnedTask, winner_id: int, now: float) -> None:
        """A specific robot acknowledged a specific GRANT. Only now is it theirs."""
        record.state = TaskState.TRANSFERRED
        record.auction = None
        record.settled_at = now
        self._counters["grants_delivered"] += 1
        self._counters["tasks_transferred"] += 1

        self._safe(lambda: self._mission.mark_transferred(record.task), None)
        self._request_yield(
            f"task {record.task.task_id} transferred to robot {winner_id}"
        )
        self._event(
            "info", f"task {record.task.task_id} is robot {winner_id}'s; confirmed"
        )
        # Normally a no-op: the task left our mission when we announced it, and
        # the transfer only confirms that was right. It still goes through the
        # helper because a GRANT can also be delivered for a task that reached
        # a winner without an announce cycle of ours.
        self._leave_mission(record, f"transferred to robot {winner_id}")

    def _grant_failed(self, record: OwnedTask, winner_id: int, now: float) -> None:
        """The winner never acknowledged. The task is still ours -- fall back.

        Three fallbacks, in order of how much they cost. Offer it to the next
        best bidder we already have; failing that, run another announce cycle
        if the budget allows; failing that, handle it locally.
        """
        record.state = TaskState.OURS
        record.granted_to = None
        record.excluded_bidders.add(int(winner_id))
        self._counters["grants_failed"] += 1
        self._event(
            "warn",
            f"GRANT for task {record.task.task_id} was not acknowledged by robot "
            f"{winner_id}; the task is still ours",
        )

        auction = record.auction
        if auction is not None:
            auction.forget(winner_id)
            runner_up = auction.winner()
            if runner_up is not None:
                record.state = TaskState.ANNOUNCED
                self._maybe_close(record, now)
                return

        record.auction = None
        if record.delegation_attempts < self.params.max_delegation_attempts:
            # A fresh cycle, so peers that were busy get another chance and the
            # cooldown does not veto our own retry of a failed delivery.
            record.last_announced_at = float("-inf")
            self._announce(record, now)
            return

        self._dispose_locally(record, record.fallback, now)

    # ------------------------------------------------------------------
    # Giving up locally
    # ------------------------------------------------------------------

    def _dispose_locally(
        self, record: OwnedTask, disposition: LocalDisposition, now: float
    ) -> None:
        """Delegation is over and the task is still ours. End it somewhere."""
        record.auction = None
        if disposition is LocalDisposition.DROP:
            record.state = TaskState.RELINQUISHED
            record.settled_at = now
            self._counters["tasks_dropped"] += 1
            self._safe(lambda: self._mission.release(record.task), None)
            self._request_yield(f"task {record.task.task_id} dropped")
            self._leave_mission(record, "dropped, undelegable")
        elif disposition is LocalDisposition.HOLD:
            # Kept, unexecuted -- and ours again. If this task went to auction
            # it left the mission when we announced it, so holding it now is a
            # real change the verifier has to be told about. (If it never
            # reached an auction the helper does nothing, which is the case the
            # old "nothing has changed" comment described.)
            record.state = TaskState.OURS
            self._counters["tasks_held"] += 1
            self._event("info", f"task {record.task.task_id} held for a later attempt")
            self._rejoin_mission(record, "held after delegation failed")
        else:  # REQUEST_HUMAN
            record.state = TaskState.OURS
            self._counters["escalated_to_human"] += 1
            self._request_yield(f"task {record.task.task_id} needs an operator")
            # Back in the mission for the same reason: it is ours, nobody else
            # is going to do it, and the operator's decision should be made
            # about a mission that still contains the work.
            self._rejoin_mission(record, "escalated to an operator")
            self._event(
                "warn",
                f"task {record.task.task_id} could not be delegated or handled; "
                f"an operator is needed",
            )

    def _take_on(self, task: Task, now: float, cause: str, note: str = "") -> None:
        """Add ``task`` to our own mission and verify what that made.

        Shared by the AddTask action and by winning someone else's auction --
        the two are the same event from different directions, and a rejected
        replan has to mean the same thing in both. ``note`` is empty for the
        first: an operator adding a task locally has no peer to have heard from.
        """
        record = self._tasks.get(task.task_id)
        if record is not None and record.ours:
            return
        self._tasks[task.task_id] = OwnedTask(task=task, state=TaskState.OURS)
        self._safe(lambda: self._mission.absorb(task), None)
        self._counters["tasks_absorbed"] += 1
        self._request_yield(f"task {task.task_id} absorbed ({cause})")

        result = self._replan(MissionDelta(added=[task], cause=cause, note=note))
        if result is not None and result.rejected:
            # The mission we just made does not verify. Hand the task straight
            # back to the anomaly path: "work we hold and cannot complete" is
            # precisely what that path exists for, and it is the one route that
            # can offer the task back to the fleet.
            self._safe(lambda: self._mission.release(task), None)
            self._event(
                "warn",
                f"replan rejected after absorbing task {task.task_id} "
                f"({result.reason}); re-interpreting as an anomaly",
            )
            self._tasks.pop(task.task_id, None)
            self.report_infeasible(
                task,
                detail=f"replan rejected: {result.reason}",
                reason_code=ReasonCode.TASK_FAILED,
            )

    # ==================================================================
    # Bidder role
    # ==================================================================

    def _on_announce(self, announce: TaskAnnounce, now: float) -> None:
        """A peer is offering work. Decide whether to answer, and how well."""
        task_id = int(announce.task_id)

        if task_id in self._bids:
            # We are already answering this one. Announcements repeat while a
            # window is open, and each copy is a distinct (src, seq) that dedup
            # will not collapse -- so idempotence has to live here.
            return
        existing = self._tasks.get(task_id)
        if existing is not None and existing.ours:
            # Our own task, announced by someone else. Not ours to bid on.
            self._counters["rx_ignored"] += 1
            return

        self._counters["bids_considered"] += 1

        if not has_capabilities(self.cap_mask, int(announce.req_cap_mask)):
            # Incapable robots stay silent. A "no" from a robot the announcer
            # never expected to hear from is pure airtime: it is not in the
            # expected-bidder set, so its answer cannot close the auction early
            # either.
            self._counters["bids_incapable"] += 1
            return

        task = Task(
            task_id=task_id,
            required_capabilities=int(announce.req_cap_mask),
            location=target_of(announce),
            priority=int(announce.priority),
        )
        fitness = self._assess(task)

        # An infeasible answer goes out at the back of the queue -- it carries
        # no fitness worth ordering -- but it does go out. Silence and "I
        # cannot" are different facts, and the wire distinguishes them so an
        # announcer can stop waiting.
        cost = fitness.cost if fitness.feasible else COST_MAX

        # Did a note about this task get here first? That is the arrival order
        # the sender aims for, and the only one that reliably works -- this
        # method is the single synchronous moment at which a bid is decided, so
        # a lookup is the only kind of question that can be asked inside it.
        note = self._note_for(int(announce.src), task_id)
        backoff = self._note_backoff if note is not None else self._backoff

        pending = PendingBid(
            task_id=task_id,
            task=task,
            announcer_id=int(announce.src),
            fitness=fitness,
            send_at=now + backoff(cost),
            created_at=now,
            deliberative=note is not None,
            note=note.text if note is not None else "",
        )
        self._bids[task_id] = pending

        if note is not None:
            # Queued, not called. Interpreting it is a model call taking
            # seconds and this runs under the lock that tick and on_message
            # need. The wider backoff above is what buys the answer time to
            # arrive before send_at.
            self._counters["notes_before_announce"] += 1
            self._request_interpretation(note, pending, now)

    # ------------------------------------------------------------------
    # Notes
    #
    # Two doors, like the anomaly path, and for the same reason: interpreting
    # a note is a model call taking seconds and it cannot run on this lock.
    # Inbound notes queue a NoteContext; the node drains the queue off the
    # lock, calls the interpreter, and comes back through revise_bid.
    # ------------------------------------------------------------------

    def on_note(self, note: CompletedNote) -> None:
        """A note arrived whole. Installed as the reliability layer's on_note.

        Never raises, for the same reason ``on_message`` never does -- this is
        one call away from the radio, and it is the only path in the system
        where another robot's *sentences* reach us.
        """
        try:
            with self._lock:
                now = self._clock()
                self._counters["notes_received"] += 1

                if int(note.src) == self.node_id:
                    self._counters["rx_ignored"] += 1
                    return

                pending = self._bids.get(int(note.task_id))
                if pending is None:
                    # No bid to revise. Either the announcement has not arrived
                    # yet -- the intended order, and _on_announce will find this
                    # note in the reliability layer's cache -- or we are not
                    # bidding on this task at all. Nothing to do either way.
                    self._counters["notes_orphaned"] += 1
                    return

                if pending.sent_at is not None:
                    # The bid is already on the air. A revision cannot be
                    # un-transmitted, so this note is a loss: it is counted,
                    # logged, and changes nothing.
                    self._counters["notes_too_late"] += 1
                    self._event(
                        "info",
                        f"note about task {note.task_id} arrived after our bid "
                        f"went out; ignored",
                    )
                    return

                self._counters["notes_after_announce"] += 1
                self._request_interpretation(note, pending, now)
        except Exception as exc:  # noqa: BLE001 - never let the radio kill us
            self._event("warn", f"handling a note raised: {exc}")

    def _note_for(self, src: int, task_id: int) -> Optional[CompletedNote]:
        """A note this announcer already finished about this task, if any."""
        getter = getattr(self._reliability, "completed_note", None)
        if getter is None:
            # A reliability layer without note support. Bidding still works;
            # it is the fast auction and always was.
            return None
        return self._safe(lambda: getter(int(src), int(task_id)), None)

    def _request_interpretation(
        self, note: CompletedNote, pending: PendingBid, now: float
    ) -> None:
        """Queue a note for interpretation. Caller holds the lock."""
        self._note_requests.append(
            NoteContext(
                text=note.text,
                task_id=int(note.task_id),
                src=int(note.src),
                task=pending.task,
                cost=int(pending.fitness.cost),
                eta_s=float(pending.fitness.eta_s),
                feasible=bool(pending.fitness.feasible),
                at=now,
            )
        )

    def take_note_requests(self) -> "list[NoteContext]":
        """Drain the notes waiting to be interpreted.

        The node calls this, runs the interpreter **off this lock**, and hands
        each answer back through :meth:`revise_bid`. Draining rather than
        peeking, so a request is dispatched once even if the node's tick and
        its inbound callback race.
        """
        with self._lock:
            requests, self._note_requests = self._note_requests, []
            return requests

    def note_interpreter(self) -> NoteInterpreter:
        """The configured interpreter, for a caller that will run it off-lock."""
        return self._interpret_note

    def revise_bid(self, task_id: int, revision: object) -> Optional[object]:
        """Apply a finished note interpretation to a bid not yet transmitted.

        Returns the revision that was applied, or None if it was refused or
        arrived too late. Validated exactly as an anomaly interpretation is --
        the closed union is the guarantee, and it does not get to be skipped by
        taking a different door.
        """
        with self._lock:
            now = self._clock()
            try:
                revision = validate_revision(revision)
            except Exception as exc:  # noqa: BLE001 - a parser that half-succeeded
                self._counters["note_interpret_failed"] += 1
                self._event("warn", f"note interpretation refused: {exc}")
                return None

            pending = self._bids.get(int(task_id))
            if pending is None or pending.sent_at is not None:
                # The auction moved on while the model was thinking. Counted
                # rather than forced: a bid already on the air is a fact, and
                # the deliberative window exists precisely so this stays rare.
                self._counters["notes_too_late"] += 1
                return None

            if isinstance(revision, KeepBid):
                self._counters["bids_kept_after_note"] += 1
                return revision

            if isinstance(revision, WithdrawBid):
                del self._bids[int(task_id)]
                self._counters["bids_withdrawn_by_note"] += 1
                self._event(
                    "info",
                    f"withdrew our bid on task {task_id} after reading the "
                    f"announcer's note: {revision.reason or 'no reason given'}",
                )
                return revision

            # ReviseBid. The delta is relative because the interpretation may
            # have run before we had an announcement to cost, so it is a claim
            # about the work rather than about our schedule.
            was = int(pending.fitness.cost)
            cost = max(0, min(was + int(revision.cost_delta), COST_MAX))
            pending.fitness = replace(pending.fitness, cost=cost)

            # Re-derive send_at from created_at on the clock this bid started
            # on, so a revised bid sits exactly where a bid of that cost would
            # have. Anchored to created_at rather than to now, because the wait
            # already served is part of the ordering the backoff encodes.
            backoff = self._note_backoff if pending.deliberative else self._backoff
            pending.send_at = max(now, pending.created_at + backoff(cost))

            self._counters["bids_revised_by_note"] += 1
            self._event(
                "info",
                f"revised our bid on task {task_id} from {was} to {cost} after "
                f"reading the announcer's note: "
                f"{revision.reason or 'no reason given'}",
            )
            return revision

    def _assess(self, task: Task) -> Fitness:
        """Ask our own nodes what this task would cost us.

        Every question is asked through a port and every answer is allowed to
        be "no". A nav that cannot answer is treated as a nav that cannot get
        there: bidding on a route we could not compute is how a task ends up
        owned by a robot that never arrives.
        """
        if not self._safe(lambda: self._nav.can_reach(task.location), False):
            return Fitness(feasible=False, reason="unreachable")
        if not self._safe(lambda: self._mission.can_absorb(task), False):
            return Fitness(feasible=False, reason="mission cannot absorb it")

        eta = self._safe(lambda: self._nav.eta(task.location), None)
        if eta is None:
            return Fitness(feasible=False, reason="no ETA available")

        current = self._safe(self._mission.current_task_id, TASK_NONE)
        return self._fitness_fn(
            task=task,
            eta_sec=float(eta),
            battery=int(self._safe(self._mission.battery_percent, 0)),
            busy=current != TASK_NONE,
        )

    def _consider_suppression(
        self, pending: PendingBid, rival: Bid, now: float
    ) -> None:
        """Overhearing a better bid during our backoff cancels ours entirely.

        This is the half of the mechanism that saves the airtime: fitness
        ordering alone would still put one packet on the air per bidder.

        Infeasible bids are exempt. Their entire purpose is to be an answer, so
        suppressing one only makes the announcer wait out a window it could
        have closed early -- and a "no" costs at most one packet from each of
        the few robots that advertise the capability.
        """
        if not pending.fitness.feasible:
            return
        if not bool(rival.feasible):
            return
        if not pending.outranked_by(
            int(rival.cost), int(rival.eta_s), int(rival.src), self.node_id
        ):
            return
        pending.best_overheard = int(rival.cost)
        del self._bids[pending.task_id]
        self._counters["bids_suppressed"] += 1
        self._event(
            "info",
            f"suppressed our bid on task {pending.task_id}: robot {rival.src} "
            f"bid {rival.cost} against our {pending.fitness.cost}",
        )

    def _transmit_bid(self, pending: PendingBid, now: float) -> None:
        self._reliability.send_broadcast(
            Bid(
                src=0,
                seq=0,
                task_id=pending.task_id,
                eta_s=quantized_eta(pending.fitness.eta_s),
                feasible=bool(pending.fitness.feasible),
                cost=(
                    min(int(pending.fitness.cost), COST_MAX)
                    if pending.fitness.feasible
                    else COST_MAX
                ),
            )
        )
        pending.sent_at = now
        self._counters["bids_sent"] += 1
        if not pending.fitness.feasible:
            self._counters["bids_infeasible"] += 1

    def _on_grant(self, grant: Grant, now: float) -> None:
        """An auction closed. Either we won it, or we can stop waiting."""
        task_id = int(grant.task_id)
        if int(grant.winner_id) != self.node_id:
            # Overheard, addressed to someone else -- which is exactly how a
            # losing bidder learns the auction is over. Reliability delivers it
            # to us un-ACKed for this purpose.
            pending = self._bids.pop(task_id, None)
            if pending is not None:
                self._counters["bids_lost"] += 1
            return

        pending = self._bids.pop(task_id, None)
        if pending is None:
            # A GRANT for a task we have no record of bidding on, and the only
            # thing it carries is a task ID -- no location, no capability, so
            # there is nothing to absorb even in principle. This is reported
            # loudly rather than swallowed because reliability has *already*
            # ACKed it: the announcer now believes this task is ours, and it
            # is not. That is an orphaned task, and the counter is what makes
            # it visible rather than mysterious.
            #
            # Reaching here means bid_memory_sec was outlived by the
            # announcer's window plus its GRANT retransmits, or a peer granted
            # a bid nobody made. The parameter check in CoordinatorParams rules
            # out the first for any single robot's own config.
            self._counters["grants_unmatched"] += 1
            self._event(
                "warn",
                f"GRANT for task {task_id} matches no bid of ours and carries no "
                f"task detail; it has been ACKed and is now orphaned",
            )
            return

        self._take_on(
            pending.task,
            now,
            cause=f"won auction from {grant.src}",
            note=pending.note,
        )

    # ==================================================================
    # Time
    # ==================================================================

    def tick(self) -> None:
        """Drive every deadline this layer owns. Call periodically; cheap when idle.

        One sweep of a few small tables rather than a timer per deadline, for
        the same reason the reliability layer does it: the tables are bounded,
        and a single sweep cannot leak a timer when something resolves early.
        """
        with self._lock:
            now = self._clock()
            self._age_peers(now)
            self._run_auctions(now)
            self._send_due_bids(now)
            self._maybe_heartbeat(now)
            self._prune(now)

    def _age_peers(self, now: float) -> None:
        for gone in self.registry.age_out(now):
            # A bid from a robot we no longer believe is there would burn a
            # whole retransmit campaign to discover as much.
            for record in self._tasks.values():
                if record.auction is not None:
                    record.auction.forget(gone.robot_id)

    def _run_auctions(self, now: float) -> None:
        for record in list(self._tasks.values()):
            auction = record.auction
            if auction is None or record.state != TaskState.ANNOUNCED:
                continue
            if now >= auction.next_announce_at and now < auction.closes_at:
                self._emit_announce(record, now)
                self._counters["re_announced"] += 1
            self._maybe_close(record, now)

    def _send_due_bids(self, now: float) -> None:
        for pending in list(self._bids.values()):
            if pending.sent_at is None and now >= pending.send_at:
                self._transmit_bid(pending, now)

    def _maybe_heartbeat(self, now: float) -> None:
        if self.params.heartbeat_period_sec <= 0 or now < self._next_heartbeat_at:
            return
        self._next_heartbeat_at = now + self.params.heartbeat_period_sec
        # Target.none() rather than a zero coordinate: "navigation has not
        # told us where we are" is a fact worth broadcasting as itself, and a
        # peer computing an ETA against a position we never claimed is worse
        # than a peer that knows our position is unknown.
        here = self._safe(self._nav.current_location, None) or Target.none()
        self._reliability.send_broadcast(
            Heartbeat(
                src=0,
                seq=0,
                cap_mask=self.cap_mask,
                **target_fields(here),
                battery=max(
                    0, min(int(self._safe(self._mission.battery_percent, 0)), 100)
                ),
                cur_task=int(self._safe(self._mission.current_task_id, TASK_NONE)),
            )
        )
        self._counters["heartbeats_sent"] += 1

    def _prune(self, now: float) -> None:
        """Forget settled tasks and stale bids, so neither table grows forever."""
        for task_id, record in list(self._tasks.items()):
            if (
                record.settled_at is not None
                and now - record.settled_at > self.params.settled_retention_sec
            ):
                del self._tasks[task_id]
        for task_id, pending in list(self._bids.items()):
            if now - pending.created_at > self.params.bid_memory_sec:
                del self._bids[task_id]

    # ==================================================================
    # Plumbing
    # ==================================================================

    def _leave_mission(self, record: OwnedTask, cause: str) -> None:
        """Take the task out of our own mission. Idempotent.

        Idempotence is the point: several paths end with the task gone --
        announced, transferred, dropped -- and they can run in sequence over
        one task. Only the first is a change.
        """
        if not record.in_mission:
            return
        record.in_mission = False
        self._replan(MissionDelta(removed=[record.task], cause=cause))

    def _rejoin_mission(self, record: OwnedTask, cause: str) -> None:
        """Put the task back into our own mission. Idempotent.

        The counterpart of announcing. An auction that fails hands the task
        back, and the mission has to be told -- otherwise the robot has quietly
        lost the work it was holding.
        """
        if record.in_mission:
            return
        record.in_mission = True
        self._replan(MissionDelta(added=[record.task], cause=cause))

    def _replan(self, delta: MissionDelta) -> Optional[ReplanResult]:
        """Call the verification point. Never lets its failure become ours."""
        self._counters["replans"] += 1
        try:
            result = self._replanner.replan_and_verify(delta)
        except Exception as exc:  # noqa: BLE001 - a solver, a subprocess, anything
            self._counters["replans_rejected"] += 1
            self._event("warn", f"replan_and_verify raised: {exc}")
            return None
        if result is not None and result.rejected:
            self._counters["replans_rejected"] += 1
        return result

    def _request_yield(self, reason: str) -> None:
        """Raise the preemption flag. Never stops anything.

        The one and only way this layer influences execution timing. There is
        deliberately no counterpart that forces an action to end -- the
        behaviour tree reads the flag at a tick point it considers safe, and an
        arm halfway through a move gets to finish the move.
        """
        self._counters["preemptions_requested"] += 1
        self._safe(lambda: self._preemption.request_yield(reason), None)

    def clear_preemption(self) -> None:
        """Lower the flag, once the behaviour tree has acted on it."""
        with self._lock:
            self._safe(self._preemption.clear, None)

    def _on_peer_change(self, event: str, record) -> None:
        self._event("info", f"peer {record.robot_id} {event}")

    def _safe(self, fn: Callable, default):
        """Call into another node's port, and survive it failing.

        Nav and the mission stack are separate processes reached over ROS. They
        restart, they time out, and none of that may take coordination down
        with them -- a fleet that stops arbitrating because one robot's
        navigation stack is reloading is worse than one that bids badly.
        """
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001 - any port, same handling
            self._event("warn", f"{getattr(fn, '__name__', 'port call')} failed: {exc}")
            return default

    def _event(self, level: str, message: str) -> None:
        if self._on_event is not None:
            self._on_event(level, message)

    # ==================================================================
    # Introspection
    # ==================================================================

    def state_of(self, task_id: int) -> Optional[TaskState]:
        with self._lock:
            record = self._tasks.get(int(task_id))
            return None if record is None else record.state

    def task(self, task_id: int) -> Optional[OwnedTask]:
        with self._lock:
            return self._tasks.get(int(task_id))

    @property
    def owned_tasks(self) -> "List[OwnedTask]":
        """Everything still ours, by the unassigned-until-ACKed definition."""
        with self._lock:
            return [record for record in self._tasks.values() if record.ours]

    @property
    def open_auctions(self) -> "List[Auction]":
        with self._lock:
            return [
                record.auction
                for record in self._tasks.values()
                if record.state == TaskState.ANNOUNCED and record.auction is not None
            ]

    @property
    def pending_bids(self) -> "List[PendingBid]":
        """Bids assessed but not yet transmitted -- still suppressible."""
        with self._lock:
            return [p for p in self._bids.values() if p.sent_at is None]

    def stats(self) -> dict:
        with self._lock:
            counters = dict(self._counters)
            counters.update(self.registry.stats())
            counters["tasks_owned"] = len(self.owned_tasks)
            counters["auctions_open"] = len(self.open_auctions)
            counters["bids_pending"] = len(self.pending_bids)
            return counters
