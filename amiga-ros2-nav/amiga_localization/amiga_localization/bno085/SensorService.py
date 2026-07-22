import time
import numpy as np

from amiga_localization.bno085.NodeParameters import NodeParameters

from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x

from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from example_interfaces.srv import Trigger


class SensorService:
    """Provide an interface for accessing the sensor's features & data."""

    def __init__(self, node: Node, connector: BNO08X_I2C, param: NodeParameters):
        self.node = node
        self.con = connector
        self.param = param

        prefix = self.param.ros_topic_prefix.value
        QoSProf = QoSProfile(depth=10)

        # create topic publishers:
        #        self.pub_imu_raw = node.create_publisher(Imu, prefix + 'imu_raw', QoSProf)
        self.pub_imu = node.create_publisher(Imu, prefix + "imu", QoSProf)
        self.pub_mag = node.create_publisher(MagneticField, prefix + "mag", QoSProf)
        # self.pub_temp = node.create_publisher(Temperature, prefix + 'temp', QoSProf)
        self.pub_calib_status = node.create_publisher(
            String, prefix + "calib_status", QoSProf
        )
        self.srv = self.node.create_service(
            Trigger, prefix + "calibration_request", self.calibration_request_callback
        )
        self.calibration_status = 0

    def configure(self):
        """Configure the IMU sensor hardware."""
        self.node.get_logger().info("Configuring device...")

        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self.con.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.con.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self.con.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
        self.con.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_STEP_COUNTER)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_STABILITY_CLASSIFIER)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_ACTIVITY_CLASSIFIER)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_SHAKE_DETECTOR)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_ACCELEROMETER)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_GYROSCOPE)
        # self.con.enable_feature(adafruit_bno08x.BNO_REPORT_RAW_MAGNETOMETER)

        self.node.get_logger().info("BNO085 IMU configuration complete.")
        if bool(self.node.param.initial_calibration.value):
            self.node.get_logger().info("Initial calibration requested.")
            self.calibrate()
        else:
            self.node.get_logger().info("Skipping initial calibration.")

    def get_sensor_data(self):
        """Read IMU data from the sensor, parse and publish."""
        if self.calibration_status < 2:
            return

        imu_msg = Imu()
        mag_msg = MagneticField()

        imu_msg.header.stamp = self.node.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.param.frame_id.value

        (
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w,
        ) = self.con.quaternion
        # todo: Normalize quaternion?
        (
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z,
        ) = self.con.linear_acceleration
        (
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z,
        ) = self.con.gyro

        imu_msg.orientation_covariance = np.diag(
            np.asarray(self.param.variance_orientation.value)
        ).flatten()
        imu_msg.linear_acceleration_covariance = np.diag(
            np.asarray(self.param.variance_acc.value)
        ).flatten()
        imu_msg.angular_velocity_covariance = np.diag(
            np.asarray(self.param.variance_angular_vel.value)
        ).flatten()
        self.pub_imu.publish(imu_msg)

        mag_msg = MagneticField()
        mag_msg.header.stamp = self.node.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.param.frame_id.value

        mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = (
            self.con.magnetic
        )  # pylint:disable=no-member
        mag_msg.magnetic_field.x = (
            mag_msg.magnetic_field.x / 1000000
        )  # convert from uT to T --> DOUBLE CHECK THIS (Sensor reports uT for sure)
        mag_msg.magnetic_field.y = mag_msg.magnetic_field.y / 1000000
        mag_msg.magnetic_field.z = mag_msg.magnetic_field.z / 1000000
        mag_msg.magnetic_field_covariance = np.diag(
            np.asarray(self.param.variance_mag.value)
        ).flatten()

        self.pub_mag.publish(mag_msg)

    def get_calib_status(self):
        """
        Read calibration status for sys/gyro/acc/mag.

        Quality scale: 0 = bad, 3 = best
        """
        # self.node.get_logger().info('Calib status')
        self.calibration_status = self.con.calibration_status
        if self.calibration_status < 2:
            self.node.get_logger().warn(
                "Calibration status is "
                + str(self.calibration_status)
                + ". Consider recalibrating the device."
            )
        else:
            self.node.get_logger().info(
                "Calibration status: " + str(self.calibration_status)
            )
        msg = String()
        msg.data = (
            "Magnetometer Calibration quality:"
            + adafruit_bno08x.REPORT_ACCURACY_STATUS[self.calibration_status]
            + " "
            + str(self.calibration_status)
        )
        self.pub_calib_status.publish(msg)

    def calibrate(self):
        # this is taken/adapted from https://github.com/adafruit/Adafruit_CircuitPython_BNO08x/blob/main/examples/bno08x_calibration.py
        self.con.begin_calibration()
        start_time = time.monotonic()
        calibration_good_at = None

        self.node.get_logger().info("Starting calibration...")

        start_calibration_time = time.monotonic()
        calibrated = False
        timeout = float(float(self.node.param.calibration_timeout.value))
        while (time.monotonic() - start_calibration_time) < timeout:
            time.sleep(0.1)

            self.calibration_status = self.con.calibration_status
            self.node.get_logger().info(
                "Calibration status:" + str(self.calibration_status)
            )

            if not calibration_good_at and self.calibration_status > 1:
                calibration_good_at = time.monotonic()

            if calibration_good_at and (time.monotonic() - calibration_good_at > 5.0):
                time.sleep(0.1)  # To reduce runtime reading errors
                self.con.save_calibration_data()
                calibrated = True
                break

        self.node.get_logger().info("Calibration done")
        if calibrated:
            return self.calibration_status
        else:
            self.node.get_logger().warn("Failed to calibrate within given deadline.")
            return -1

    def calibration_request_callback(self, request, response):
        calibration_status = self.calibrate()
        if calibration_status == -1:
            response.success = False
            response.message = "Calibration timedout"
        else:
            response.success = True
            response.message = "Calibration result: " + str(calibration_status)
        return response
