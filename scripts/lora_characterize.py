#!/usr/bin/env python3
"""Characterize the real LoRa link by timing GRANT/ACK round trips.

Standalone: this brings up its own copy of the two nodes that already exist
for this -- ``LoRaBridge`` (owns the serial port to the radio) and
``ReliabilityNode`` (the ACK/retransmit engine a real auction's GRANT uses)
-- in this one process. It does not assume ``lora_bridge``, a coordinator, or
anything else is already running; there is nothing else to launch first,
which is also why device selection lives here now: pass the radio's serial
port and this robot's fleet ID directly.

Two machines, two radios, one script, role picked at the command line::

    # fixed end -- e.g. staying at the aisle head
    python3 scripts/lora_characterize.py responder \\
        --serial-port /dev/ttyUSB0 --node-id 2

    # end that walks away
    python3 scripts/lora_characterize.py initiator \\
        --serial-port /dev/ttyUSB0 --node-id 1 --peer-id 2 \\
        --csv /tmp/lora_walk.csv

Why GRANT/ACK and not HEARTBEAT: HEARTBEAT is broadcast and never
acknowledged (see amiga_ros2_comms/codec/messages.py and
reliability/addressing.py -- reliability follows addressing, and broadcast is
never reliable), and it carries no send time, so "receive it on the other
machine" alone can only show whether it arrived, not how long it took -- and
the two machines have no synchronised clock to diff against even if it did.
GRANT is the one message type this stack ever sends reliably: the initiator's
``ReliabilitySession.send_reliable()`` stamps it, sends it, and resolves a
Future only once the ACK comes back (or the retransmit budget runs out) --
see amiga_ros2_comms/reliability/session.py. Timing that Future measures the
same round trip a real auction's GRANT takes to close, through the same
retransmit/backoff logic, over the real radio. The responder side needs no
code of its own: any ReliabilityNode auto-ACKs a GRANT addressed to its
node_id, which is *why* the node.py module docstring says it "runs standalone
... enough to be useful on a bench with two radios."

The initiator logs every attempt (delivered, with its RTT, or failed, once
the retry budget is exhausted) to a CSV. While it runs, type a number and
Enter at any time to relabel subsequent samples with the distance you just
paced off:

    50<Enter>      # from here on, samples are logged at distance_m=50

Then, offline and without ROS:

    python3 scripts/lora_characterize.py plot --csv /tmp/lora_walk.csv
"""

import argparse
import csv
import os
import signal
import sys
import threading
import time
from datetime import datetime, timezone

#: CoordinatorParams.announce_window_sec's default (see
#: amiga_ros2_coordinator/engine/coordinator.py). Hardcoded, not imported --
#: `plot` is meant to also run on a laptop with just the CSV and no ROS
#: workspace built. Override with --announce-window-sec if the fleet is
#: configured differently.
DEFAULT_ANNOUNCE_WINDOW_SEC = 5.0

CSV_COLUMNS = [
    "seq",
    "wall_time_utc",
    "distance_m",
    "node_id",
    "peer_id",
    "outcome",
    "rtt_ms",
]


def _open_csv(path: str):
    is_new = not os.path.exists(path) or os.path.getsize(path) == 0
    f = open(path, "a", newline="")
    writer = csv.writer(f)
    if is_new:
        writer.writerow(CSV_COLUMNS)
        f.flush()
    return f, writer


def _bridge_params(args) -> dict:
    return {
        "serial_port": args.serial_port,
        "baud": args.baud,
        "max_payload_bytes": args.max_payload_bytes,
        # We print our own combined status line (see _status_line); the
        # node's built-in one logs at DEBUG, which is invisible by default.
        "stats_period_sec": 0.0,
    }


def _reliability_params(args) -> dict:
    return {
        "node_id": args.node_id,
        "retransmit_timeout_sec": args.retransmit_timeout_sec,
        "max_retries": args.max_retries,
        "retransmit_backoff": args.retransmit_backoff,
        "max_retransmit_timeout_sec": args.max_retransmit_timeout_sec,
        "max_payload_bytes": args.max_payload_bytes,
        "spreading_factor": args.spreading_factor,
        "stats_period_sec": 0.0,
    }


def _make_bridge_and_reliability(args):
    # Imported here, not at module scope, so `plot` needs no ROS workspace.
    from rclpy.parameter import Parameter

    from amiga_ros2_comms.lora.bridge_node import LoRaBridge
    from amiga_ros2_comms.reliability.node import ReliabilityNode

    bridge = LoRaBridge(
        parameter_overrides=[
            Parameter(k, value=v) for k, v in _bridge_params(args).items()
        ]
    )
    reliability = ReliabilityNode(
        parameter_overrides=[
            Parameter(k, value=v) for k, v in _reliability_params(args).items()
        ]
    )
    return bridge, reliability


def _status_line(bridge, reliability) -> str:
    b, r = bridge.stats(), reliability.stats()
    return (
        f"port_open={b.get('port_open')} rx_frames={r.get('rx_frames')} "
        f"tx_delivered={r.get('tx_delivered')} tx_failed={r.get('tx_failed')} "
        f"tx_retransmits={r.get('tx_retransmits')} rx_acks={r.get('rx_acks')}"
    )


def _spin(nodes, executor) -> None:
    from rclpy.executors import ExternalShutdownException

    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        for node in nodes:
            shutdown = getattr(node, "shutdown", None)
            if shutdown is not None:
                shutdown()
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()


def run_responder(args) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    rclpy.init()
    bridge, reliability = _make_bridge_and_reliability(args)
    reliability.get_logger().info(
        f"responder up as node_id={args.node_id} on {args.serial_port}: "
        "every GRANT addressed to this id is auto-ACKed by the reliability "
        "layer, so there is nothing else for this role to do -- see "
        "amiga_ros2_comms/reliability/session.py."
    )
    reliability.create_timer(
        args.status_period_sec,
        lambda: print(_status_line(bridge, reliability), flush=True),
    )

    executor = MultiThreadedExecutor(num_threads=4)
    nodes = (bridge, reliability)
    for node in nodes:
        executor.add_node(node)
    try:
        _spin(nodes, executor)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def run_initiator(args) -> None:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.node import Node

    from amiga_ros2_comms.codec import Grant

    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    rclpy.init()
    bridge, reliability = _make_bridge_and_reliability(args)

    csv_file, writer = _open_csv(args.csv)
    lock = threading.Lock()
    state = {"distance_m": args.distance_m, "seq": 0, "sent": 0, "recv": 0}

    def current_distance():
        with lock:
            return state["distance_m"]

    def read_stdin():
        for line in sys.stdin:
            line = line.strip()
            if not line:
                continue
            try:
                value = float(line)
            except ValueError:
                print(f"not a number, ignored: {line!r}", flush=True)
                continue
            with lock:
                state["distance_m"] = value
            print(f"distance now {value:g} m", flush=True)

    def on_result(seq, distance, sent_t, future):
        rtt_ms = (time.monotonic() - sent_t) * 1000.0
        outcome = future.result()
        with lock:
            state["recv"] += 1
            recv, sent = state["recv"], state["sent"]
        writer.writerow(
            [
                seq,
                datetime.now(timezone.utc).isoformat(),
                distance,
                reliability.node_id,
                args.peer_id,
                outcome.value,
                f"{rtt_ms:.2f}",
            ]
        )
        csv_file.flush()
        print(
            f"seq={seq:6d} distance={distance!s:>6}m {outcome.value:9s} "
            f"rtt={rtt_ms:7.1f}ms ({recv}/{sent} delivered-or-resolved)",
            flush=True,
        )

    driver = Node("lora_char_driver")

    def on_timer():
        with lock:
            seq = state["seq"]
            state["seq"] += 1
            state["sent"] += 1
        distance = current_distance()
        # src/seq are overwritten by send_reliable; task_id is ours to use as
        # the correlation id these bare GRANTs don't otherwise carry.
        grant = Grant(src=0, seq=0, task_id=seq & 0xFFFF, winner_id=args.peer_id)
        sent_t = time.monotonic()
        future = reliability.send_reliable(args.peer_id, grant)
        future.add_done_callback(
            lambda f, seq=seq, distance=distance, sent_t=sent_t: on_result(
                seq, distance, sent_t, f
            )
        )

    driver.create_timer(args.interval_sec, on_timer)
    driver.create_timer(
        args.status_period_sec,
        lambda: print(_status_line(bridge, reliability), flush=True),
    )
    threading.Thread(target=read_stdin, daemon=True).start()

    reliability.get_logger().info(
        f"initiator up as node_id={args.node_id}, targeting peer_id={args.peer_id} "
        f"every {args.interval_sec}s via a real GRANT + ACK round trip "
        f"(retransmit_timeout={args.retransmit_timeout_sec}s "
        f"x{args.retransmit_backoff} up to {args.max_retries} retries -- same "
        "path and tuning a live auction's GRANT would use). Type a number + "
        "Enter any time to relabel following samples with a new paced-off "
        "distance."
    )

    executor = MultiThreadedExecutor(num_threads=4)
    nodes = (bridge, reliability, driver)
    for node in nodes:
        executor.add_node(node)
    try:
        _spin(nodes, executor)
    finally:
        csv_file.close()
        if rclpy.ok():
            rclpy.shutdown()


def _cmd_plot(args) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rtt_by_distance = {}
    outcomes_by_distance = {}
    with open(args.csv, newline="") as f:
        for row in csv.DictReader(f):
            try:
                distance = float(row["distance_m"])
            except (KeyError, ValueError):
                continue
            outcomes_by_distance.setdefault(distance, [0, 0])
            outcomes_by_distance[distance][1] += 1
            if row.get("outcome") == "delivered":
                rtt = row.get("rtt_ms")
                if rtt:
                    rtt_by_distance.setdefault(distance, []).append(float(rtt))
            else:
                outcomes_by_distance[distance][0] += 1

    if not outcomes_by_distance:
        print(f"no rows found in {args.csv}", file=sys.stderr)
        sys.exit(1)

    distances = sorted(outcomes_by_distance)
    means = [
        (
            sum(rtt_by_distance[d]) / len(rtt_by_distance[d])
            if rtt_by_distance.get(d)
            else None
        )
        for d in distances
    ]

    print(f"{'distance_m':>10} {'n_ok':>6} {'n_failed':>9} {'mean_rtt_ms':>12}")
    for d in distances:
        samples = rtt_by_distance.get(d, [])
        failed, total = outcomes_by_distance[d]
        mean = sum(samples) / len(samples) if samples else float("nan")
        print(f"{d:>10g} {len(samples):>6d} {failed:>9d} {mean:>12.1f}")

    fig, (ax_rtt, ax_loss) = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

    for d in distances:
        samples = rtt_by_distance.get(d, [])
        if samples:
            ax_rtt.scatter(
                [d] * len(samples), samples, s=10, alpha=0.4, color="tab:blue"
            )
    plotted = [(d, m) for d, m in zip(distances, means) if m is not None]
    if plotted:
        ax_rtt.plot(
            *zip(*plotted), color="tab:blue", marker="o", label="mean RTT (delivered)"
        )
    ax_rtt.axhline(
        args.announce_window_sec * 1000.0,
        color="tab:red",
        linestyle="--",
        label=f"auction announce window ({args.announce_window_sec:g}s)",
    )
    ax_rtt.set_ylabel("GRANT round-trip time (ms)")
    ax_rtt.set_title("LoRa GRANT/ACK round-trip latency vs. distance")
    ax_rtt.legend()
    ax_rtt.grid(True, alpha=0.3)

    fail_pct = [
        100.0 * outcomes_by_distance[d][0] / outcomes_by_distance[d][1]
        for d in distances
    ]
    span = (max(distances) - min(distances)) if len(distances) > 1 else 1.0
    ax_loss.bar(distances, fail_pct, width=max(1.0, span / 40), color="tab:orange")
    ax_loss.set_ylabel("failed (%)\n(retry budget exhausted)")
    ax_loss.set_xlabel("distance (m)")
    ax_loss.grid(True, alpha=0.3)

    fig.tight_layout()
    out = args.out or os.path.splitext(args.csv)[0] + ".png"
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


def _add_common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument(
        "--serial-port",
        required=True,
        help="device path of this machine's LoRa radio, e.g. /dev/ttyUSB0",
    )
    p.add_argument(
        "--node-id",
        type=int,
        required=True,
        help="this robot's fleet-unique id, 1..255",
    )
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--max-payload-bytes", type=int, default=200)
    p.add_argument(
        "--spreading-factor",
        type=int,
        default=7,
        help="only used to sanity-check --retransmit-timeout-sec against the radio's airtime",
    )
    p.add_argument("--retransmit-timeout-sec", type=float, default=3.0)
    p.add_argument("--max-retries", type=int, default=3)
    p.add_argument("--retransmit-backoff", type=float, default=1.5)
    p.add_argument("--max-retransmit-timeout-sec", type=float, default=10.0)
    p.add_argument("--status-period-sec", type=float, default=10.0)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = parser.add_subparsers(dest="mode", required=True)

    init_p = sub.add_parser(
        "initiator", help="sends GRANTs, times the ACK, logs RTT to CSV"
    )
    _add_common_args(init_p)
    init_p.add_argument(
        "--peer-id", type=int, required=True, help="the responder's --node-id"
    )
    init_p.add_argument(
        "--interval-sec", type=float, default=5.0, help="seconds between attempts"
    )
    init_p.add_argument(
        "--distance-m", type=float, default=0.0, help="starting distance label"
    )
    init_p.add_argument(
        "--csv", default="lora_characterize.csv", help="log file (appended to)"
    )

    resp_p = sub.add_parser("responder", help="auto-ACKs every GRANT addressed to it")
    _add_common_args(resp_p)

    plot_p = sub.add_parser(
        "plot", help="plot RTT vs. distance from a CSV (no ROS needed)"
    )
    plot_p.add_argument("--csv", required=True, help="CSV produced by `initiator`")
    plot_p.add_argument(
        "--out", default=None, help="output image path (default: <csv>.png)"
    )
    plot_p.add_argument(
        "--announce-window-sec",
        type=float,
        default=DEFAULT_ANNOUNCE_WINDOW_SEC,
        help="reference line: the auction's announce window",
    )

    args = parser.parse_args()
    if args.mode == "plot":
        _cmd_plot(args)
    elif args.mode == "initiator":
        run_initiator(args)
    else:
        run_responder(args)


if __name__ == "__main__":
    main()
