#!/usr/bin/env python3

import cv2
import rclpy
from sensor_msgs.msg import Image 
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration

class ColorDetection: 
    def __init__(self): 
        self.get_logger().info("Initializeing COlor detection")
        self.image_sub = self.create_subscription(Image, "/camera/rgb/image_raw", self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, data): 
        self.get_logger().info("image received")
        
        try: 
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.get_logger().info("Image converted to OpenCV format...")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Process the image for color detection
        self.detect_colors(cv_image)

    def detect_colors(self, frame):
        self.get_logger().info("Processing image for color detection...")

        # Smooth the image to reduce noise
        blurred_frame = cv2.GaussianBlur(frame, (5, 5), 0)

        # Convert the image to HSV
        hsv = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)

        # Define color ranges for detection
        color_ranges = {
            "red": ([0, 120, 70], [10, 255, 255]),
            "green": ([36, 50, 70], [89, 255, 255]),
            "blue": ([90, 50, 70], [128, 255, 255]),
            "yellow": ([20, 100, 100], [30, 255, 255]),
            "purple": ([129, 50, 70], [158, 255, 255])
        }

        kernel = np.ones((5, 5), np.uint8)

        for color_name, (lower, upper) in color_ranges.items():
            lower = np.array(lower, dtype="uint8")
            upper = np.array(upper, dtype="uint8")

            # Create a mask for the color
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours of the detected color regions
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                # Get bounding box around the contour
                x, y, w, h = cv2.boundingRect(contour)
                if cv2.contourArea(contour) > 1000:  # Filter small areas
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(frame, color_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Show the processed image
        self.get_logger().info("Displaying processed image...")
        cv2.imshow("Color Detection", frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutting down...")
            cv2.destroyAllWindows()
            self.get_logger().info("User requested shutdown.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

