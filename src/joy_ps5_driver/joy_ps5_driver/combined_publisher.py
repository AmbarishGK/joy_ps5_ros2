import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from joy_ps5_interfaces.msg import Motion, Touchpad, DualSenseExt
from std_msgs.msg import Header

class DualSenseAggregator(Node):
    def __init__(self):
        super().__init__('dual_sense_aggregator')

        # Create a publisher for DualSenseExt message
        self.publisher_ = self.create_publisher(DualSenseExt, 'dual_sense_ext_topic', 10)

        # Create subscribers for Joy, Motion, and Touchpad data
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.motion_subscription = self.create_subscription(
            Motion,
            'motion_data',
            self.motion_callback,
            10
        )
        self.touchpad_subscription = self.create_subscription(
            Touchpad,
            'touchpad_data',
            self.touchpad_callback,
            10
        )

        # Create a timer to periodically check and publish the message
        self.timer = self.create_timer(0.1, self.publish_combined_message)

        # Variables to store received messages
        self.joy_data = None
        self.motion_data = None
        self.touchpad_data = None

    def joy_callback(self, msg):
        """Callback function for Joy data."""
        self.joy_data = msg
        self.get_logger().info(f"Received Joy data: {msg.axes}, {msg.buttons}")

    def motion_callback(self, msg):
        """Callback function for Motion data."""
        self.motion_data = msg
        self.get_logger().info(f"Received Motion data: "
                               f"Accel: ({msg.accel_x}, {msg.accel_y}, {msg.accel_z}), "
                               f"Gyro: ({msg.gyro_x}, {msg.gyro_y}, {msg.gyro_z})")

    def touchpad_callback(self, msg):
        """Callback function for Touchpad data."""
        self.touchpad_data = msg
        self.get_logger().info(f"Received Touchpad data: x: {msg.x}, y: {msg.y}, "
                               f"Touch Detected: {msg.touch_detected}, Tracking ID: {msg.tracking_id}")

    def publish_combined_message(self):
        """Publish the combined message (DualSenseExt) if all data is received."""
        if self.joy_data and self.motion_data and self.touchpad_data:
            # Create a new DualSenseExt message
            msg = DualSenseExt()

            # Populate the Joy data
            msg.joy_data = self.joy_data

            # Populate the Motion data
            msg.motion_data = self.motion_data

            # Populate the Touchpad data
            msg.touch_data = self.touchpad_data

            # Set the header
            msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")

            # Publish the message
            self.publisher_.publish(msg)
            self.get_logger().info(f"Publishing combined DualSenseExt message: {msg}")

def main(args=None):
    rclpy.init(args=args)

    dual_sense_aggregator = DualSenseAggregator()

    rclpy.spin(dual_sense_aggregator)

    dual_sense_aggregator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
