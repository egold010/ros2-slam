#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select

# Control keys and corresponding velocities
MOVE_BINDINGS = {
    'w': (0.5, 0.0),  # Forward
    's': (-0.5, 0.0),  # Backward
    'a': (0.0, 1.0),  # Rotate left
    'd': (0.0, -1.0),  # Rotate right
    'x': (0.0, 0.0),  # Stop
}

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Keyboard teleop node started. Use W/A/S/D to move, X to stop. CTRL+C to quit.")

    def get_key(self):
        """Non-blocking key reader with timeout."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        """Main loop that reads keypresses and publishes Twist messages."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in MOVE_BINDINGS:
                    linear_x, angular_z = MOVE_BINDINGS[key]
                    self.publish_velocity(linear_x, angular_z)
                elif key == '\x03':  # Ctrl+C to exit
                    break
                else:
                    linear_x, angular_z = MOVE_BINDINGS['x']
                    self.publish_velocity(linear_x, angular_z)
        except Exception as e:
            self.get_logger().error(f"Exception in teleop: {e}")
        finally:
            # Stop the robot before exiting
            self.publish_velocity(0.0, 0.0)

    def publish_velocity(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent command - linear: {linear_x}, angular: {angular_z}')

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)  # Save terminal settings

    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    node.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
