#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
import math
import random

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('marker_publisher')
        self.pub = self.create_publisher(Marker, 'green_dot', 10)
        self.timer = self.create_timer(0.05, self.publish_marker)  # 20 Hz

        self.x = random.uniform(-2.0, 2.0)
        self.y = random.uniform(-2.0, 2.0)
        self.speed = 0.1
        self.set_new_direction(bias_toward_center=True)

        self.steps_in_direction = 0
        self.max_steps = 100

    def set_new_direction(self, bias_toward_center=False):
        if bias_toward_center:
            # Vector pointing toward center
            dx = -self.x
            dy = -self.y
            base_angle = math.atan2(dy, dx)

            # Add small random variation 
            angle = base_angle + random.uniform(-math.pi / 4, math.pi / 4)
        else:
            angle = random.uniform(0, 2 * math.pi)

        self.vx = self.speed * math.cos(angle)
        self.vy = self.speed * math.sin(angle)

    def publish_marker(self):
        # Update position
        self.x += self.vx
        self.y += self.vy

        # Bounce if outside bounds and reset direction
        bounced = False
        if self.x < -5 or self.x > 5:
            self.vx *= -1
            bounced = True
        if self.y < -5 or self.y > 5:
            self.vy *= -1
            bounced = True

        if bounced:
            self.steps_in_direction = 0
            self.set_new_direction(bias_toward_center=True)

        # Periodically change direction toward center
        self.steps_in_direction += 1
        if self.steps_in_direction > self.max_steps:
            self.set_new_direction(bias_toward_center=True)
            self.steps_in_direction = 0

        # Create and publish marker
        now = self.get_clock().now()
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = now.to_msg()
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.1
        marker.color.g = 0.8
        marker.color.b = 0.3
        marker.color.a = 1.0

        marker.lifetime = Duration()
        self.pub.publish(marker)
        self.get_logger().info(f"publishing marker at: {marker.pose.position.x:.3f}, {marker.pose.position.y:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
