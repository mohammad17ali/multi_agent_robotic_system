import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.distance_to_move = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def move_robot(self):
        # Publish the velocity command to move the robot
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.angular.z = self.angular_velocity
        self.publisher_.publish(twist)

    def get_input(self):
        # Read the user input for distance to move
        self.distance_to_move = float(input("Enter the distance to move: "))
        # Read the user input for linear velocity
        self.linear_velocity = float(input("Enter the linear velocity: "))
        # Read the user input for angular velocity
        self.angular_velocity = float(input("Enter the angular velocity: "))

    def wait_for_space_key(self):
        # Wait for the user to press the space key
        print("Press the space key to move the robot...")
        tty.setcbreak(sys.stdin.fileno())  # Set terminal to cbreak mode
        char = sys.stdin.read(1)
        if char == ' ':
            print("Moving the robot...")
            return True
        return False

    def move_robot_distance(self):
        # Move the robot the specified distance
        self.move_robot()
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.move_robot()
        rclpy.spin_once(self)
        rclpy.spin_once(self)
        rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)
    controller = TurtleBotController()

    while True:
        controller.get_input()
        if controller.wait_for_space_key():
            controller.move_robot_distance()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
