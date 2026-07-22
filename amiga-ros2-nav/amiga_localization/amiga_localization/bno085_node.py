# Partially based on https://github.com/flynneva/bno055/blob/main/bno055/bno055.py

import sys
import threading

import board
from adafruit_bno08x.i2c import BNO08X_I2C
import rclpy
from rclpy.node import Node

from amiga_localization.bno085.SensorService import SensorService
from amiga_localization.bno085.NodeParameters import NodeParameters


class Bno085Node(Node):
    """
    ROS2 Node for interfacing Bosch Bno085 IMU sensor.

    :param Node: ROS2 Node Class to initialize from
    :type Node: ROS2 Node
    :raises NotImplementedError: Indicates feature/function is not implemented yet.
    """

    sensor = None
    param = None

    def __init__(self):
        # Initialize parent (ROS Node)
        super().__init__("bno085")

    def setup(self):
        # Initialize ROS2 Node Parameters:
        self.param = NodeParameters(self)

        # Get connector according to configured sensor connection type:
        if self.param.connection_type.value == "i2c":
            connector = BNO08X_I2C(board.I2C())
            connector.initialize()  # should be already called in constructor, though...
        else:
            raise NotImplementedError(
                "Unsupported connection type: " + str(self.param.connection_type.value)
            )

        # Instantiate the sensor Service API:
        self.sensor = SensorService(self, connector, self.param)

        # configure imu
        self.sensor.configure()


def main(args=None):
    try:
        """Main entry method for this ROS2 node."""
        # Initialize ROS Client Libraries (RCL) for Python:
        rclpy.init()

        # Create & initialize ROS2 node:
        node = Bno085Node()
        node.setup()

        # Create lock object to prevent overlapping data queries
        lock = threading.Lock()

        def read_data():
            """Periodic data_query_timer executions to retrieve sensor IMU data."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn(
                    "Message communication in progress - skipping query cycle"
                )
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_sensor_data()
            except ZeroDivisionError:
                # division by zero in get_sensor_data, return
                return
            except Exception as e:  # noqa: B902
                node.get_logger().warn(
                    'Receiving sensor data failed with %s:"%s"' % (type(e).__name__, e)
                )
            finally:
                lock.release()

        def log_calibration_status():
            """Periodic logging of calibration data (quality indicators)."""
            if lock.locked():
                # critical area still locked
                # that means that the previous data query is still being processed
                node.get_logger().warn(
                    "Message communication in progress - skipping query cycle"
                )
                # traceback.print_exc()
                return

            # Acquire lock before entering critical area to prevent overlapping data queries
            lock.acquire()
            try:
                # perform synchronized block:
                node.sensor.get_calib_status()
            except Exception as e:  # noqa: B902
                node.get_logger().warn(
                    'Receiving calibration status failed with %s:"%s"'
                    % (type(e).__name__, e)
                )
                # traceback.print_exc()
            finally:
                lock.release()

        # start regular sensor transmissions:
        # please be aware that frequencies around 30Hz and above might cause performance impacts:
        # https://github.com/ros2/rclpy/issues/520
        f = 1.0 / float(node.param.data_query_frequency.value)
        data_query_timer = node.create_timer(f, read_data)

        # start regular calibration status logging
        f = 1.0 / float(node.param.calib_status_frequency.value)
        status_timer = node.create_timer(f, log_calibration_status)

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received - exiting...")
        sys.exit(0)
    finally:
        node.get_logger().info("ROS node shutdown")
        try:
            node.destroy_timer(data_query_timer)
            node.destroy_timer(status_timer)
        except UnboundLocalError:
            node.get_logger().info("No timers to shutdown")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
