import rclpy
from rclpy.node import Node
from evdev import InputDevice, ecodes, list_devices
from std_msgs.msg import Header
from joy_ps5_interfaces.msg import Motion  # Replace with your actual package and message name

class MotionSensorPublisher(Node):
    def __init__(self):
        super().__init__('motion_sensor_publisher')
        self.device = self.find_motion_sensor_device()  # Automatically find the motion sensor device
        if not self.device:
            self.get_logger().error("No motion sensor device found.")
            return

        self.publisher = self.create_publisher(Motion, 'motion_data', 10)
        self.values = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.get_logger().info(f"Listening to device: {self.device.name}")
        self.read_device_loop()

    def find_motion_sensor_device(self):
        # List all input devices and check for the motion sensor
        devices = [InputDevice(path) for path in list_devices()]
        for device in devices:
            if "DualSense Wireless Controller Motion Sensors" in device.name:
                return device
        return None

    def read_device_loop(self):
        try:
            for event in self.device.read_loop():
                # Process EV_ABS events for codes 0-5 (motion sensors)
                if event.type == ecodes.EV_ABS and event.code in range(6):
                    self._update_values(event.code, event.value)
                
                # Process EV_SYN to publish data
                elif event.type == ecodes.EV_SYN:
                    self.publish_data(event.timestamp())
        except Exception as e:
            self.get_logger().error(f"An error occurred: {str(e)}")

    def _update_values(self, code, value):
        if code == 0:
            self.values['x'] = value / 8192.0  # Normalize based on resolution
        elif code == 1:
            self.values['y'] = value / 8192.0
        elif code == 2:
            self.values['z'] = value / 8192.0
        elif code == 3:
            self.values['rx'] = (value / 1024.0) * (3.14159 / 180.0)  # Convert to radians
        elif code == 4:
            self.values['ry'] = (value / 1024.0) * (3.14159 / 180.0)
        elif code == 5:
            self.values['rz'] = (value / 1024.0) * (3.14159 / 180.0)

    def publish_data(self, timestamp):
        msg = Motion()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS clock for consistent timestamps
        msg.header.frame_id = "motion_sensor"

        msg.accel_x = self.values['x']
        msg.accel_y = self.values['y']
        msg.accel_z = self.values['z']
        msg.gyro_x = self.values['rx']
        msg.gyro_y = self.values['ry']
        msg.gyro_z = self.values['rz']

        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg}")

def main(args=None):
    rclpy.init(args=args)
    motion_sensor_publisher = MotionSensorPublisher()  # Now automatically detects the device
    if motion_sensor_publisher.device:
        rclpy.spin(motion_sensor_publisher)
        motion_sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
