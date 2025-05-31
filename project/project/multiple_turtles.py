#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
import random
import time

class MultiRandomTurtleMover(Node):
    def __init__(self):
        super().__init__('multi_random_turtle_mover')

        self.turtle_names = ['turtle2', 'turtle3', 'turtle4']
        self.turtle_publishers = {}

        # Set up spawn client
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')

        # Begin spawning
        self.current = 0
        self.spawn_next_turtle()

    def spawn_next_turtle(self):
        if self.current >= len(self.turtle_names):
            self.get_logger().info('All turtles spawned. Starting movement...')
            self.start_timer()
            return

        turtle_name = self.turtle_names[self.current]
        req = Spawn.Request()
        req.x = random.uniform(1.0, 9.0)
        req.y = random.uniform(1.0, 9.0)
        req.theta = random.uniform(0, 6.28)
        req.name = turtle_name

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.on_spawned)

    def on_spawned(self, future):
        try:
            response = future.result()
            name = response.name
            self.get_logger().info(f"Spawned {name}")
            self.turtle_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
        except Exception as e:
            self.get_logger().error(f"Error spawning turtle: {e}")

        self.current += 1
        time.sleep(0.5)
        self.spawn_next_turtle()

    def start_timer(self):
        self.timer = self.create_timer(0.5, self.move_all_turtles)

    def move_all_turtles(self):
        for name, pub in self.turtle_publishers.items():
            twist = Twist()
            twist.linear.x = random.uniform(0.5, 2.0)
            twist.angular.z = random.uniform(-2.0, 2.0)
            pub.publish(twist)
            self.get_logger().info(f"{name}: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = MultiRandomTurtleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
