#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class RandomTurtleMover(Node):
    def __init__(self):
        super().__init__('random_turtle_mover')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.send_random_velocity)  # Change velocity every 0.5 seconds
        
    def send_random_velocity(self):
        twist = Twist()

        # Random linear velocity (forward/backward)
        twist.linear.x = random.uniform(0.5, 2.0)    # you can make this negative for backward movement

        # Random angular velocity (turn left or right)
        twist.angular.z = random.uniform(-2.0, 2.0)

        self.publisher.publish(twist)
        self.get_logger().info(f"Moving turtle with linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomTurtleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
