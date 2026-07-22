#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import threading


class GPSPlotter(Node):
    def __init__(self):
        super().__init__("gps_plotter")

        # Parameters (set your own topic/image/map bounds here)
        self.declare_parameter("topic", "/ublox_gps_node/fix")
        self.declare_parameter("image_path", "map.png")
        self.declare_parameter("lat_min", 37.366099319129866)
        self.declare_parameter("lat_max", 37.36646456984841)
        self.declare_parameter("lon_min", -120.42330393665887)
        self.declare_parameter("lon_max", -120.42285518393685)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        self.subscription = self.create_subscription(
            NavSatFix, topic, self.gps_callback, 10
        )

        # Load map image
        image_path = self.get_parameter("image_path").get_parameter_value().string_value
        self.img = mpimg.imread(image_path)
        self.lat_min = self.get_parameter("lat_min").get_parameter_value().double_value
        self.lat_max = self.get_parameter("lat_max").get_parameter_value().double_value
        self.lon_min = self.get_parameter("lon_min").get_parameter_value().double_value
        self.lon_max = self.get_parameter("lon_max").get_parameter_value().double_value

        # Store GPS points
        self.lats = []
        self.lons = []

        # Setup matplotlib
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(
            self.img, extent=[self.lon_min, self.lon_max, self.lat_min, self.lat_max]
        )
        (self.path_line,) = self.ax.plot([], [], "r-", linewidth=2)
        self.ax.set_xlabel("Longitude")
        self.ax.set_ylabel("Latitude")
        self.ax.set_title("Live GPS Track")
        self.fig.canvas.draw()

        # Use a ROS timer instead of a thread
        self.timer = self.create_timer(0.1, self.update_plot)

    def gps_callback(self, msg: NavSatFix):
        if not (msg.latitude and msg.longitude):
            return
        self.lats.append(msg.latitude)
        self.lons.append(msg.longitude)
        self.get_logger().info(f"GPS: {msg.latitude:.6f}, {msg.longitude:.6f}")

    def update_plot(self):
        """Update the plot safely on the main thread."""
        if len(self.lats) > 1:
            self.path_line.set_data(self.lons, self.lats)
            self.ax.set_xlim(self.lon_min, self.lon_max)
            self.ax.set_ylim(self.lat_min, self.lat_max)
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = GPSPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show(block=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
