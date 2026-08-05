#!/usr/bin/env python3
"""ROS2 node hosting a virtual LoRa medium, for Gazebo and other radio-less runs.

Run one of these per simulation. It creates a serial device per robot and
carries frames between them over a simulated air channel; each robot then runs
its ordinary lora_bridge pointed at its own device. No bridge parameter changes
and no code changes: as far as the bridge is concerned it has a radio.

    ros2 launch amiga_ros2_comms lora_sim.launch.py robots:=robot1,robot2,robot3

This node is the sim harness, not part of the robot stack. Nothing on a real
robot should ever launch it, and nothing else should depend on it existing.
"""

from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .airtime import DWELL_LIMIT_SEC, RadioConfig, airtime_sec, max_payload_for_dwell
from .virtual_medium import VirtualLoRaMedium

# Where the per-robot device paths are created. Stable names, because a launch
# file has to name a port before the pty behind it exists.
DEFAULT_SYMLINK_DIR = "/tmp/amiga_lora_sim"


def _describe(text: str) -> ParameterDescriptor:
    return ParameterDescriptor(description=text)


class LoRaSim(Node):
    """Owns the virtual medium and exposes its knobs as ROS parameters."""

    def __init__(self, **node_kwargs):
        super().__init__("lora_sim", **node_kwargs)
        self._declare_parameters()

        robots = [str(name) for name in self.get_parameter("robots").value]
        radio = RadioConfig(
            spreading_factor=int(self.get_parameter("spreading_factor").value),
            bandwidth_hz=int(self.get_parameter("bandwidth_hz").value),
            coding_rate=int(self.get_parameter("coding_rate").value),
            preamble_symbols=int(self.get_parameter("preamble_symbols").value),
        )
        self._dwell_limit = float(self.get_parameter("dwell_limit_sec").value)

        self._medium = VirtualLoRaMedium(
            names=robots,
            symlink_dir=self.get_parameter("symlink_dir").value,
            radio=radio,
            dwell_limit_sec=self._dwell_limit,
            on_event=self._log_event,
        )

        stats_period = float(self.get_parameter("stats_period_sec").value)
        self._stats_timer = (
            self.create_timer(stats_period, self._log_stats)
            if stats_period > 0
            else None
        )
        self._announce(radio)

    def _declare_parameters(self) -> None:
        self.declare_parameter(
            "robots",
            ["robot1", "robot2"],
            _describe("Names of the robots on the air. One serial device each."),
        )
        self.declare_parameter(
            "symlink_dir",
            DEFAULT_SYMLINK_DIR,
            _describe(
                "Directory holding one stable symlink per robot, pointing at its "
                "pty. Each bridge's serial_port is <symlink_dir>/<robot>."
            ),
        )
        self.declare_parameter(
            "spreading_factor",
            7,
            _describe(
                "LoRa spreading factor, 6..12. Drives time on air, and with it "
                "how much traffic the fleet can actually sustain."
            ),
        )
        self.declare_parameter(
            "bandwidth_hz", 125000, _describe("LoRa channel bandwidth in Hz.")
        )
        self.declare_parameter(
            "coding_rate", 5, _describe("Denominator of the 4/N coding rate, 5..8.")
        )
        self.declare_parameter(
            "preamble_symbols", 8, _describe("Preamble length in symbols.")
        )
        self.declare_parameter(
            "dwell_limit_sec",
            DWELL_LIMIT_SEC,
            _describe(
                "Warn once if a frame's time on air exceeds this. Defaults to "
                "the FCC Part 15.247 400 ms limit. 0 disables the check."
            ),
        )
        self.declare_parameter(
            "stats_period_sec",
            30.0,
            _describe("Period of the counters log line. 0 disables it."),
        )

    # ------------------------------------------------------------------
    # Diagnostics and lifecycle
    # ------------------------------------------------------------------

    def _announce(self, radio: RadioConfig) -> None:
        budget = max_payload_for_dwell(radio, self._dwell_limit or DWELL_LIMIT_SEC)
        log = self.get_logger()
        log.info(
            f"virtual lora medium at {radio.describe()}: "
            f"{len(self._medium.ports)} radios, "
            f"200 B payload = {airtime_sec(200, radio) * 1000:.0f} ms on air, "
            f"dwell budget allows {budget} B"
        )
        for name, path in self._medium.ports.items():
            log.info(f"  {name}: serial_port:={path}")

    def _log_event(self, level: str, message: str) -> None:
        # Distinct call sites: rclpy refuses to log two severities from one
        # line. See the same note in bridge_node._log_link_event.
        if level == "warn":
            self.get_logger().warn(message)
        else:
            self.get_logger().info(message)

    def stats(self) -> dict:
        return self._medium.stats()

    def _log_stats(self) -> None:
        self.get_logger().info(
            " ".join(f"{k}={v}" for k, v in sorted(self.stats().items()))
        )

    def shutdown(self) -> None:
        self._medium.close()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = LoRaSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
