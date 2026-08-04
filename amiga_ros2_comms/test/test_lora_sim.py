#!/usr/bin/env python3
"""The simulator driving real bridge nodes, which is the case Gazebo cares about.

Three unmodified lora_bridge instances, one virtual radio each, talking over a
simulated channel. If these pass, a Gazebo fleet can use the same code and the
same topics it will use on hardware.
"""

import time

import pytest
from rclpy.parameter import Parameter

from amiga_ros2_comms.lora.airtime import RadioConfig
from amiga_ros2_comms.lora.sim_node import LoRaSim
from amiga_ros2_comms.lora.virtual_medium import VirtualLoRaMedium
from ros_harness import Collector, Harness, make_bridge, wait_until

# Faster than the 125 kHz default so the ROS round trip dominates rather than
# the airtime; the airtime model itself is covered in test_virtual_medium.py.
FAST_RADIO = RadioConfig(spreading_factor=7, bandwidth_hz=500_000)


@pytest.fixture
def medium_factory():
    made = []

    def build(names, **kwargs):
        kwargs.setdefault("radio", FAST_RADIO)
        medium = VirtualLoRaMedium(names, **kwargs)
        made.append(medium)
        return medium

    yield build
    for medium in made:
        medium.close()


def test_a_fleet_of_unmodified_bridges_talks_over_the_simulated_air(medium_factory):
    names = ["robot1", "robot2", "robot3"]
    medium = medium_factory(names)
    ports = medium.ports

    bridges = [make_bridge(ports[name], name) for name in names]
    listeners = {
        name: Collector("robot1", name, name=f"listener_{name}") for name in names
    }
    harness = Harness(*bridges, *listeners.values())
    try:
        assert wait_until(lambda: all(b.stats()["port_open"] for b in bridges))
        time.sleep(0.5)  # let the subscriptions match before the first publish

        payload = b"task 7 claimed by robot1"
        listeners["robot1"].send(payload)

        assert wait_until(
            lambda: listeners["robot2"].payloads() and listeners["robot3"].payloads()
        )
        # One publish on /robot1/lora/tx, heard by everyone else on their own
        # /lora/rx: broadcast, with no addressing anywhere in the bridge.
        assert listeners["robot2"].payloads() == [payload]
        assert listeners["robot3"].payloads() == [payload]

        time.sleep(0.5)
        assert listeners["robot1"].payloads() == [], "a radio must not hear itself"
    finally:
        harness.close()


def test_every_robot_can_transmit(medium_factory):
    names = ["robot1", "robot2"]
    medium = medium_factory(names)
    ports = medium.ports

    bridges = [make_bridge(ports[name], name) for name in names]
    one_to_two = Collector("robot1", "robot2", name="one_to_two")
    two_to_one = Collector("robot2", "robot1", name="two_to_one")
    harness = Harness(*bridges, one_to_two, two_to_one)
    try:
        assert wait_until(lambda: all(b.stats()["port_open"] for b in bridges))
        time.sleep(0.5)

        one_to_two.send(b"robot1 here")
        assert wait_until(lambda: one_to_two.payloads() == [b"robot1 here"])
        two_to_one.send(b"robot2 here")
        assert wait_until(lambda: two_to_one.payloads() == [b"robot2 here"])
    finally:
        harness.close()


def test_a_bridge_started_before_the_sim_connects_once_the_port_appears(
    medium_factory, tmp_path
):
    """Launch order must not matter.

    Under ros2 launch there is no ordering guarantee between the sim and the
    robots' bringup, so a bridge that starts first has to wait rather than die.
    """
    directory = str(tmp_path / "lora")
    names = ["robot1", "robot2"]

    bridges = [
        make_bridge(f"{directory}/{name}", name, reconnect_period_sec=0.3)
        for name in names
    ]
    listener = Collector("robot1", "robot2", name="late_listener")
    harness = Harness(*bridges, listener)
    try:
        time.sleep(0.5)
        assert not any(b.stats()["port_open"] for b in bridges), "no ports exist yet"

        medium_factory(names, symlink_dir=directory)

        assert wait_until(
            lambda: all(b.stats()["port_open"] for b in bridges), timeout=15.0
        )
        time.sleep(0.5)
        listener.send(b"late but present")
        assert wait_until(lambda: listener.payloads() == [b"late but present"])
    finally:
        harness.close()


def test_the_sim_node_exposes_the_medium_through_parameters(tmp_path):
    directory = str(tmp_path / "lora")
    node = LoRaSim(
        parameter_overrides=[
            Parameter("robots", value=["alpha", "beta"]),
            Parameter("symlink_dir", value=directory),
            Parameter("spreading_factor", value=9),
            Parameter("stats_period_sec", value=0.0),
        ]
    )
    try:
        assert set(node._medium.ports) == {"alpha", "beta"}
        assert node._medium.ports["alpha"] == f"{directory}/alpha"
        assert node._medium.radio_config.spreading_factor == 9
        assert node.stats()["frames_sent"] == 0
    finally:
        node.shutdown()
        node.destroy_node()
