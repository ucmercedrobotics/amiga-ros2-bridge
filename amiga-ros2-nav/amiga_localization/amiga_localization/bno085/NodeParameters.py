# PUT LICENSE AND CREDITS

from rclpy.node import Node


DEFAULT_VARIANCE_ACC = [0.017, 0.017, 0.017]
DEFAULT_VARIANCE_ANGULAR_VEL = [0.04, 0.04, 0.04]
DEFAULT_VARIANCE_ORIENTATION = [0.0159, 0.0159, 0.0159]
DEFAULT_VARIANCE_NOT_CALIB = [100.0, 100.0, 100.0]
# TODO(flynneva) calculate default magnetic variance matrice
DEFAULT_VARIANCE_MAG = [0.0, 0.0, 0.0]


class NodeParameters:
    """
    ROS2 Node Parameter Handling.

    based off https://github.com/flynneva/bno055/blob/main/bno055/params/NodeParameters.py

    https://index.ros.org/doc/ros2/Tutorials/Parameters/Understanding-ROS2-Parameters
    https://index.ros.org/doc/ros2/Tutorials/Using-Parameters-In-A-Class-Python/

    Start the node with parameters from yml file:
    ros2 run amiga_localization bno085

    with the following arguments:
    --ros-args --params-file install/amiga_localization/share/amiga_localization/config/bno085_params.yaml
    """

    def __init__(self, node: Node):
        node.get_logger().info("Initializing parameters")
        # Declare parameters of the ROS2 node and their default values:

        # The topic prefix to use (can be empty if not required)
        node.declare_parameter(name="ros_topic_prefix", value="bno085/")
        # The type of the sensor connection. Currently supports only i2c
        node.declare_parameter(name="connection_type", value="i2c")
        # tf frame id
        node.declare_parameter("frame_id", value="bno085")
        # Node timer frequency in Hz, defining how often sensor data is requested
        node.declare_parameter("data_query_frequency", value=10)
        # Node timer frequency in Hz, defining how often calibration status data is requested
        node.declare_parameter("calib_status_frequency", value=1)
        # perform initial calibration?
        node.declare_parameter("initial_calibration", value=False)
        node.declare_parameter("calibration_timeout", value=30)

        # sensor operation mode
        #        node.declare_parameter('operation_mode', value=0x0C)
        # scaling factor for acceleration
        #       node.declare_parameter('acc_factor', value=100.0)
        # scaling factor for magnetometer
        #      node.declare_parameter('mag_factor', value=16000000.0)
        # scaling factor for gyroscope
        #     node.declare_parameter('gyr_factor', value=900.0)
        # determines whether to use default offsets or not
        #    node.declare_parameter('set_offsets', value=False)
        # +/- 2000 units (at max 2G) (1 unit = 1 mg = 1 LSB = 0.01 m/s2)
        #   node.declare_parameter('offset_acc', value=registers.DEFAULT_OFFSET_ACC)
        # +/- 6400 units (1 unit = 1/16 uT)
        #  node.declare_parameter('offset_mag', value=registers.DEFAULT_OFFSET_MAG)
        # +/- 2000 units up to 32000 (dps range dependent)               (1 unit = 1/16 dps)
        # node.declare_parameter('offset_gyr', value=registers.DEFAULT_OFFSET_GYR)
        # +/-1000 units
        #  node.declare_parameter('radius_acc', value=registers.DEFAULT_RADIUS_ACC)
        #  +/-960 units
        #  node.declare_parameter('radius_mag', value=registers.DEFAULT_RADIUS_MAG)
        # Sensor standard deviation squared (^2) defaults [x, y, z]
        node.declare_parameter("variance_acc", value=DEFAULT_VARIANCE_ACC)
        node.declare_parameter(
            "variance_angular_vel", value=DEFAULT_VARIANCE_ANGULAR_VEL
        )
        node.declare_parameter(
            "variance_orientation", value=DEFAULT_VARIANCE_ORIENTATION
        )
        node.declare_parameter(
            "variance_mag", value=DEFAULT_VARIANCE_MAG
        )  # THIS NEEDS TO BE TUNED!!
        node.declare_parameter("variance_not_calib", value=DEFAULT_VARIANCE_NOT_CALIB)

        # get the parameters - requires CLI arguments '--ros-args --params-file <parameter file>'
        node.get_logger().info("Parameters set to:")

        try:
            self.ros_topic_prefix = node.get_parameter("ros_topic_prefix")
            node.get_logger().info(
                '\tros_topic_prefix:\t"%s"' % self.ros_topic_prefix.value
            )

            self.connection_type = node.get_parameter("connection_type")
            node.get_logger().info(
                '\tconnection_type:\t"%s"' % self.connection_type.value
            )

            self.frame_id = node.get_parameter("frame_id")
            node.get_logger().info('\tframe_id:\t\t"%s"' % self.frame_id.value)

            self.data_query_frequency = node.get_parameter("data_query_frequency")
            node.get_logger().info(
                '\tdata_query_frequency:\t"%s"' % self.data_query_frequency.value
            )

            self.calib_status_frequency = node.get_parameter("calib_status_frequency")
            node.get_logger().info(
                '\tcalib_status_frequency:\t"%s"' % self.calib_status_frequency.value
            )

            self.initial_calibration = node.get_parameter("initial_calibration")
            node.get_logger().info(
                '\tinitial_calibration:\t"%s"' % self.initial_calibration.value
            )

            self.calibration_timeout = node.get_parameter("calibration_timeout")
            node.get_logger().info(
                '\tcalibration_timeout:\t"%s"' % self.calibration_timeout.value
            )

            self.variance_acc = node.get_parameter("variance_acc")
            node.get_logger().info('\tvariance_acc:\t\t"%s"' % self.variance_acc.value)

            self.variance_angular_vel = node.get_parameter("variance_angular_vel")
            node.get_logger().info(
                '\tvariance_angular_vel:\t"%s"' % self.variance_angular_vel.value
            )

            self.variance_orientation = node.get_parameter("variance_orientation")
            node.get_logger().info(
                '\tvariance_orientation:\t"%s"' % self.variance_orientation.value
            )

            self.variance_mag = node.get_parameter("variance_mag")
            node.get_logger().info('\tvariance_mag:\t\t"%s"' % self.variance_mag.value)

            self.variance_not_calib = node.get_parameter("variance_not_calib")
            node.get_logger().info(
                '\tvariance_not_calib:\t"%s"' % self.variance_not_calib.value
            )

        except Exception as e:
            node.get_logger().warn(
                "Could not get parameters...setting variables to default"
            )
            node.get_logger().warn('Error: "%s"' % e)
