import rclpy
from rclpy.node import Node
from joy_ps5_interfaces.msg import Motion

class Tester(Node):
    def __init__(self):
        super().__init__('tester')
        self.get_logger().info('Tester node started')
        self.create_timer(1, self.timer_callback)
        self.counter = 0
    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Counter: {self.counter}')  
def main(args=None):
    rclpy.init(args=args)
    tester = Tester()
    rclpy.spin(tester)
    tester.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()