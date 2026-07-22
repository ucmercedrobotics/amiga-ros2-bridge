import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class ImuOrientationPlotter(Node):
    def __init__(self):
        super().__init__("imu_orientation_plotter")

        self.declare_parameter("imu_topic", "bno085/imu")
        self.declare_parameter("mag_topic", "bno085/mag")
        self.declare_parameter("history_secs", 60.0)
        self.declare_parameter("max_points", 6000)
        self.declare_parameter("use_header_stamp", True)

        self.imu_topic = self.get_parameter("imu_topic").value
        self.mag_topic = self.get_parameter("mag_topic").value
        self.history_secs = float(self.get_parameter("history_secs").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.use_header_stamp = bool(self.get_parameter("use_header_stamp").value)

        qos = QoSProfile(depth=50)
        self.sub = self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos)
        self.mag_sub = self.create_subscription(
            MagneticField, self.mag_topic, self._mag_cb, qos
        )

        self._t0 = None
        self._lock = threading.Lock()
        self._t = deque()
        self._roll = deque()
        self._pitch = deque()
        self._yaw = deque()
        self._qnorm = deque()
        self._mag_t = deque()
        self._mag_x = deque()
        self._mag_y = deque()

    def _imu_cb(self, msg: Imu):
        if self.use_header_stamp and msg.header.stamp.sec != 0:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            now = self.get_clock().now().to_msg()
            t = now.sec + now.nanosec * 1e-9

        if self._t0 is None:
            self._t0 = t
        t = t - self._t0

        qx = float(msg.orientation.x)
        qy = float(msg.orientation.y)
        qz = float(msg.orientation.z)
        qw = float(msg.orientation.w)

        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if qnorm > 0.0:
            qx /= qnorm
            qy /= qnorm
            qz /= qnorm
            qw /= qnorm

        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)

        with self._lock:
            self._t.append(t)
            self._roll.append(math.degrees(roll))
            self._pitch.append(math.degrees(pitch))
            self._yaw.append(math.degrees(yaw))
            self._qnorm.append(qnorm)

            self._trim_history()

    def _trim_history(self):
        while len(self._t) > self.max_points:
            self._t.popleft()
            self._roll.popleft()
            self._pitch.popleft()
            self._yaw.popleft()
            self._qnorm.popleft()

        if self.history_secs > 0 and len(self._t) > 1:
            newest = self._t[-1]
            while len(self._t) > 1 and (newest - self._t[0]) > self.history_secs:
                self._t.popleft()
                self._roll.popleft()
                self._pitch.popleft()
                self._yaw.popleft()
                self._qnorm.popleft()

        while len(self._mag_t) > self.max_points:
            self._mag_t.popleft()
            self._mag_x.popleft()
            self._mag_y.popleft()

        if self.history_secs > 0 and len(self._mag_t) > 1:
            newest = self._mag_t[-1]
            while len(self._mag_t) > 1 and (newest - self._mag_t[0]) > self.history_secs:
                self._mag_t.popleft()
                self._mag_x.popleft()
                self._mag_y.popleft()

    def snapshot(self):
        with self._lock:
            return (
                list(self._t),
                list(self._roll),
                list(self._pitch),
                list(self._yaw),
                list(self._qnorm),
                list(self._mag_t),
                list(self._mag_x),
                list(self._mag_y),
            )

    def _mag_cb(self, msg: MagneticField):
        if self.use_header_stamp and msg.header.stamp.sec != 0:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            now = self.get_clock().now().to_msg()
            t = now.sec + now.nanosec * 1e-9

        if self._t0 is None:
            self._t0 = t
        t = t - self._t0

        mx = float(msg.magnetic_field.x)
        my = float(msg.magnetic_field.y)

        with self._lock:
            self._mag_t.append(t)
            self._mag_x.append(mx)
            self._mag_y.append(my)
            self._trim_history()


def quaternion_to_euler(x: float, y: float, z: float, w: float):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def main():
    rclpy.init()
    node = ImuOrientationPlotter()

    fig, ax = plt.subplots(1, 1, figsize=(6, 6))
    fig.suptitle("Magnetometer Distortion (XY Circle View)")

    (line_mag,) = ax.plot([], [], label="mag xy")
    (point_mag,) = ax.plot([], [], "o", label="latest")
    (ref_circle,) = ax.plot([], [], "--", alpha=0.5, label="ref circle")
    ax.set_xlabel("mag x")
    ax.set_ylabel("mag y")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")

    def update(_):
        (
            t,
            roll,
            pitch,
            yaw,
            qnorm,
            mag_t,
            mag_x,
            mag_y,
        ) = node.snapshot()
        if not mag_x:
            return line_mag, point_mag, ref_circle

        line_mag.set_data(mag_x, mag_y)
        point_mag.set_data([mag_x[-1]], [mag_y[-1]])

        radii = [math.hypot(x, y) for x, y in zip(mag_x, mag_y)]
        if radii:
            r = sorted(radii)[len(radii) // 2]
        else:
            r = 1.0
        theta = [i * 2.0 * math.pi / 200.0 for i in range(201)]
        ref_circle.set_data([r * math.cos(a) for a in theta], [r * math.sin(a) for a in theta])

        max_abs = max(max(abs(v) for v in mag_x), max(abs(v) for v in mag_y), r, 1e-6)
        lim = max_abs * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        return line_mag, point_mag, ref_circle

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    anim = FuncAnimation(fig, update, interval=100, blit=False)
    try:
        plt.show()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if anim:
            pass


if __name__ == "__main__":
    main()
