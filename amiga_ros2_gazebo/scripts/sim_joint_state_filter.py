#!/usr/bin/env python3
"""Filter the sim-wide /joint_states stream down to a named joint subset.

The Gazebo controller manager publishes a mixed /joint_states stream for the
whole robot. This node can republish either the Kinova arm/gripper joints or
the Amiga wheel joints, depending on the selected mode.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


FILTERS = {
    "kinova": {
        "output_topic": "/kinova/joint_states",
        "joint_names": [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
            "robotiq_85_left_knuckle_joint",
            "robotiq_85_right_knuckle_joint",
            "robotiq_85_left_inner_knuckle_joint",
            "robotiq_85_right_inner_knuckle_joint",
            "robotiq_85_left_finger_tip_joint",
            "robotiq_85_right_finger_tip_joint",
        ],
        "rename_map": {},
    },
    "amiga": {
        "output_topic": "/amiga/joint_states",
        "joint_names": [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint",
        ],
        "rename_map": {
            "front_left_wheel_joint": "front_left_wheel_link_joint",
            "front_right_wheel_joint": "front_right_wheel_link_joint",
            "rear_left_wheel_joint": "rear_left_wheel_link_joint",
            "rear_right_wheel_joint": "rear_right_wheel_link_joint",
        },
    },
}


class SimJointStateFilter(Node):
    def __init__(self):
        super().__init__("sim_joint_state_filter")
        self.declare_parameter("input_topic", "/joint_states")
        self.declare_parameter("mode", "kinova")

        mode = str(self.get_parameter("mode").value)
        config = FILTERS.get(mode)
        if config is None:
            raise ValueError(f"Unsupported filter mode: {mode}")

        self.output_topic = config["output_topic"]
        self.joint_names = list(config["joint_names"])
        self.rename_map = dict(config["rename_map"])

        self.pub = self.create_publisher(JointState, self.output_topic, 10)
        self.sub = self.create_subscription(
            JointState, self.get_parameter("input_topic").value, self.cb, 10
        )

    def cb(self, msg: JointState):
        index_by_name = {name: idx for idx, name in enumerate(msg.name)}

        filtered = JointState()
        filtered.header = msg.header

        for field in ("name", "position", "velocity", "effort"):
            values = getattr(msg, field)
            if not values:
                continue

            selected = []
            for joint_name in self.joint_names:
                idx = index_by_name.get(joint_name)
                if idx is not None and idx < len(values):
                    selected.append(values[idx])
            setattr(filtered, field, selected)

        filtered.name = [
            self.rename_map.get(name, name)
            for name in self.joint_names
            if name in index_by_name
        ]
        self.pub.publish(filtered)


def main():
    rclpy.init()
    rclpy.spin(SimJointStateFilter())


if __name__ == "__main__":
    main()