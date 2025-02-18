import rclpy
from rclpy.node import Node
import serial
import sys
import termios
import tty

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_publisher')
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Ensure correct port
        self.get_logger().info("Motor Controller Node Started. Press 'w/a/s/d' to move. Press 'q' to quit.")

    def get_key(self):
        """Capture single key press"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            if key == 'q':  # Quit on 'q'
                self.get_logger().info("Exiting...")
                break
            elif key in ['w', 'a', 's', 'd']:
                self.serial_port.write(key.encode())  # Send keypress to Arduino
                self.get_logger().info(f"Sent command: {key}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

