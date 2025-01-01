import rclpy
from rclpy.node import Node
from evdev import InputDevice, ecodes, list_devices
from std_msgs.msg import Header
from joy_ps5_interfaces.msg import Touchpad  # Replace with your actual package and message name

class TouchpadPublisher(Node):
    def __init__(self):
        super().__init__('touchpad_publisher')
        self.device = self.find_touchpad_device()  # Automatically find the touchpad device
        if not self.device:
            self.get_logger().error("No touchpad device found.")
            return

        self.publisher = self.create_publisher(Touchpad, 'touchpad_data', 10)
        self.get_logger().info(f"Listening to device: {self.device.name}")
        self.read_device_loop()

    def find_touchpad_device(self):
        # List all input devices and check for the touchpad
        devices = [InputDevice(path) for path in list_devices()]
        for device in devices:
            if "DualSense Wireless Controller Touchpad" in device.name:
                return device
        return None

    def read_device_loop(self):
        try:
            touch_data = {
                'x': 0.0,
                'y': 0.0,
                'tracking_id': 0,
                'mt_x': 0.0,
                'mt_y': 0.0,
                'touch_detected': False,
                'double_tap_detected': False,
                'left_button_pressed': False,
                'finger_tool_detected': False
            }

            for event in self.device.read_loop():
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        touch_data['x'] = event.value / 1919.0  # Normalize based on max value
                    elif event.code == ecodes.ABS_Y:
                        touch_data['y'] = event.value / 1079.0  # Normalize based on max value
                    elif event.code == ecodes.ABS_MT_POSITION_X:
                        touch_data['mt_x'] = event.value / 1919.0
                    elif event.code == ecodes.ABS_MT_POSITION_Y:
                        touch_data['mt_y'] = event.value / 1079.0
                    elif event.code == ecodes.ABS_MT_TRACKING_ID:
                        # Ensure tracking_id is an unsigned integer within the valid range 0-4294967295
                        touch_data['tracking_id'] = max(0, min(4294967295, event.value))  # Clamp to 0-4294967295

                elif event.type == ecodes.EV_KEY:
                    if event.code == ecodes.BTN_TOUCH:
                        touch_data['touch_detected'] = event.value == 1
                    elif event.code == ecodes.BTN_TOOL_DOUBLETAP:
                        touch_data['double_tap_detected'] = event.value == 1
                    elif event.code == ecodes.BTN_LEFT:
                        touch_data['left_button_pressed'] = event.value == 1
                    elif event.code == ecodes.BTN_TOOL_FINGER:
                        touch_data['finger_tool_detected'] = event.value == 1

                elif event.type == ecodes.EV_SYN:
                    self.publish_data(touch_data)

        except Exception as e:
            self.get_logger().error(f"An error occurred: {str(e)}")

    def publish_data(self, touch_data):
        msg = Touchpad()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Use ROS clock for consistent timestamps
        msg.header.frame_id = "touchpad"

        msg.x = touch_data['x']
        msg.y = touch_data['y']
        msg.tracking_id = touch_data['tracking_id']
        msg.mt_x = touch_data['mt_x']
        msg.mt_y = touch_data['mt_y']
        msg.touch_detected = touch_data['touch_detected']
        msg.double_tap_detected = touch_data['double_tap_detected']
        msg.left_button_pressed = touch_data['left_button_pressed']
        msg.finger_tool_detected = touch_data['finger_tool_detected']

        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {msg}")

def main(args=None):
    rclpy.init(args=args)
    touchpad_publisher = TouchpadPublisher()  # Now automatically detects the device
    if touchpad_publisher.device:
        rclpy.spin(touchpad_publisher)
        touchpad_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
