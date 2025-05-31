
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from project_msg.srv import TurtleFollower
import math
import random

class Turtle(Node):
    def __init__(self):
        super().__init__('mixed_turtle_controller')

        self.leader_name = 'turtle1'
        self.random_turtles = ['turtle2', 'turtle3', 'turtle4']
        self.follower_name = 'turtle5'
        self.all_turtles = [self.leader_name] + self.random_turtles + [self.follower_name]

        self.poses = {}
        self.turtle_publishers = {}

        # Spawn client
        self.spawn_client = self.create_client(Spawn, '/spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        # Create pose subscriptions and publishers
        for name in self.all_turtles:
            self.create_subscription(Pose, f'/{name}/pose', self.make_pose_callback(name), 10)
            self.turtle_publishers[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)

        # Start spawning turtles
        self.turtles_to_spawn = self.random_turtles + [self.follower_name]
        self.spawn_next()

        # Timers
        self.create_timer(0.5, self.move_random)
        self.create_timer(0.5, self.move_leader)
        self.create_timer(0.1, self.follow_target)

        # Set initial follow target
        self.current_target = self.leader_name

        # Create service to change follow target
        self.create_service(TurtleFollower, 'set_follow_target', self.follow_target_callback)

    def make_pose_callback(self, name):
        def callback(msg):
            self.poses[name] = msg
        return callback

    def spawn_next(self):
        if not self.turtles_to_spawn:
            return
        name = self.turtles_to_spawn.pop(0)
        req = Spawn.Request()
        req.x = random.uniform(2.0, 8.0)
        req.y = random.uniform(2.0, 8.0)
        req.theta = 0.0
        req.name = name
        future = self.spawn_client.call_async(req)
        future.add_done_callback(lambda f: self.spawn_next())

    def move_random(self):
        margin = 1.0
        for name in self.random_turtles:
            if name not in self.poses:
                continue

            pose = self.poses[name]
            twist = Twist()

            if pose.x < margin or pose.x > 11 - margin or pose.y < margin or pose.y > 11 - margin:
                twist.linear.x = 1.5
                twist.angular.z = 2.0 # Turn around
            else:
                twist.linear.x = random.uniform(0.5, 2.0)
                twist.angular.z = random.uniform(-2.0, 2.0)

            self.turtle_publishers[name].publish(twist)

    def move_leader(self):
        if self.leader_name not in self.poses:
            return

        pose = self.poses[self.leader_name]
        margin = 1.0
        twist = Twist()

        if pose.x < margin or pose.x > 11 - margin or pose.y < margin or pose.y > 11 - margin:
            twist.linear.x = 1.5
            twist.angular.z = 2.0
        else:
            twist.linear.x = random.uniform(0.5, 2.0)
            twist.angular.z = random.uniform(-2.0, 2.0)

        self.turtle_publishers[self.leader_name].publish(twist)

    def follow_target(self):
        if self.current_target not in self.poses or self.follower_name not in self.poses:
            return

        target = self.poses[self.current_target]
        follower = self.poses[self.follower_name]

        dx = target.x - follower.x
        dy = target.y - follower.y
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - follower.theta)

        twist = Twist()
        twist.linear.x = 1.5
        twist.angular.z = 4.0 * angle_diff
        self.turtle_publishers[self.follower_name].publish(twist)

    def follow_target_callback(self, request, response):
        new_target = request.name
        if new_target not in self.all_turtles:
            response.success = False
            response.message = f"Turtle '{new_target}' is not valid."
            self.get_logger().warn(response.message)
        else:
            self.current_target = new_target
            response.success = True
            response.message = f"Now following: {new_target}"
            self.get_logger().info(response.message)
        return response


    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = Turtle()
    rclpy.spin(node)
    rclpy.shutdown()
