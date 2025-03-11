#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import os
import sys
import uuid

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        self.bridge = CvBridge()

        # Path to Conda environment Python
        self.conda_python = '/home/faheem/anaconda3/envs/fyp/bin/python'

        # Path to the detect_apriltag script
        self.detect_script = '/home/faheem/IRobot/src/inventory/subprocess/detect_apriltag.py'

        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("AprilTag Capture Node Ready")

    def image_callback(self, msg):
        """Receives ROS image, saves it, and calls detection script"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save to a temporary file
            temp_filename = f'/tmp/{uuid.uuid4()}.png'
            cv2.imwrite(temp_filename, cv_image)

            # Call the AprilTag detection script
            result = subprocess.run(
                [self.conda_python, self.detect_script, temp_filename],
                capture_output=True,
                text=True
            )

            # Check response from `detect_apriltag.py`
            if result.returncode == 0 and result.stdout.strip() == "done":
                self.get_logger().info(f"AprilTag detected and saved successfully.")
            elif result.stdout.strip() == "no tags found":
                self.get_logger().info("No AprilTags detected.")
            else:
                self.get_logger().error(f"Unexpected response: {result.stdout.strip()}")
            
            # Shutdown the ROS node and exit the script
            self.get_logger().info("Shutting down after detection.")
            rclpy.shutdown()
            sys.exit(0)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
