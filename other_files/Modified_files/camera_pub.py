#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Camera configuration
        self.device = '/dev/video0'  # Update if needed (e.g., '/dev/video1')
        self.width = 320
        self.height = 240
        self.fps = 30
        self.fourcc = 'MJPG'  # Compressed format for better performance

        # Initialize camera
        self.cap = None
        self.init_camera()

        # Create a timer to publish frames at self.fps
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def init_camera(self):
        """Initialize the camera with error handling."""
        try:
            self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            time.sleep(2)  # Warmup delay for hardware initialization

            if not self.cap.isOpened():
                raise RuntimeError(f"Failed to open camera device: {self.device}")

            # Set camera parameters (with possible error checking)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
            if not self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width):
                self.get_logger().warn(f"Could not set width to {self.width}")
            if not self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height):
                self.get_logger().warn(f"Could not set height to {self.height}")
            if not self.cap.set(cv2.CAP_PROP_FPS, self.fps):
                self.get_logger().warn(f"Could not set FPS to {self.fps}")

            # Log actual settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().info(f"Camera initialized: {actual_width}x{actual_height} @ {actual_fps} FPS")

        except Exception as e:
            self.get_logger().fatal(f"Camera init failed: {str(e)}")
            raise

    def timer_callback(self):
        """Capture and publish frames."""
        try:
            # Reinitialize if the camera got disconnected
            if not self.cap or not self.cap.isOpened():
                self.get_logger().warn("Camera not initialized or lost. Attempting reinit...")
                self.init_camera()
                return

            ret, frame = self.cap.read()
            if not ret or frame is None:
                self.get_logger().warn('Frame capture failed', throttle_duration_sec=5)
                return

            # Flip frame if needed (-1 flips both horizontally & vertically)
            frame = cv2.flip(frame, -1)

            # Convert the OpenCV image to a ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Camera error: {str(e)}')

    def destroy_node(self):
        """Clean up camera resources."""
        self.get_logger().info('Releasing camera resources')
        if self.cap and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()








