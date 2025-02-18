import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorCommandPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        self.timer = self.create_timer(2.0, self.send_command)
        self.speed = 100
        self.direction = 'F'

    def send_command(self):
        command = f'{self.direction} {self.speed}'
        self.publisher_.publish(String(data=command))
        self.get_logger().info(f'Published: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

