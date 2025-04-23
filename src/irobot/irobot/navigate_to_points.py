import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import sys
import uuid
from datetime import datetime
from pymongo import MongoClient

def main():
    rclpy.init()

    # Create an instance of the navigator
    navigator = BasicNavigator()
    node = rclpy.create_node('robot_controller')

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.000
    initial_pose.pose.position.y = 0.000
    initial_yaw = 0.0
    z, w = compute_quaternion_from_yaw(initial_yaw)
    initial_pose.pose.orientation.z = z
    initial_pose.pose.orientation.w = w
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to activate
    navigator.waitUntilNav2Active()
    
    # Define waypoints (example coordinates - modify as needed)
    waypoints = [
        
          {"label": "p1", "x": 0.0, "y": 0.0, "yaw": 0.0}, # up 
          {"label": "p2", "x": 1.03, "y": 0.0, "yaw": 0.0}, # up 
          {"label": "p3", "x": 1.85, "y": 0.0, "yaw": 0.0}, # up
          {"label": "s1", "x": 2.2, "y": -1.3, "yaw":  math.pi }, # down 
          {"label": "p4", "x": 1.25, "y": -1.3, "yaw": math.pi},# down
          {"label": "p5", "x": 0.51, "y": -1.3, "yaw": math.pi}, # down
          {"label": "p6", "x": -0.2, "y": -1.3, "yaw": math.pi}, # down
        #   {"label": "p7", "x": -0.2, "y": -1.3, "yaw": 0.0}, # up
        #   {"label": "p8", "x": 0.51, "y": -1.3, "yaw": 0.0}, # down
        #   {"label": "p9", "x": 1.25, "y": -1.3, "yaw": 0.0},# down
    ]

    current_waypoint_index = 0

    while rclpy.ok() and current_waypoint_index < len(waypoints):
        waypoint = waypoints[current_waypoint_index]
        z, w = compute_quaternion_from_yaw(waypoint["yaw"])
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = waypoint["x"]
        goal_pose.pose.position.y = waypoint["y"]
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w

        # Start navigation
        print(f"Navigating to {waypoint['label']} at ({waypoint['x']}, {waypoint['y']})")
        navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not navigator.isTaskComplete():
            rclpy.spin_once(node, timeout_sec=0.1)

        # Handle navigation result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Successfully reached {waypoint['label']}")
            
            
            
            
            ########## capture code will work here ################
            
            
            
            
            
            current_waypoint_index += 1
        else:
            print(f"Failed to reach {waypoint['label']}. Skipping...")
            current_waypoint_index += 1

    print("Navigation completed.")
    
    ############# missing items to only be updated in the end ################
    
    
    rclpy.shutdown()

def compute_quaternion_from_yaw(yaw):
    """Compute quaternion from yaw angle (radians)"""
    return math.sin(yaw/2), math.cos(yaw/2)

if __name__ == "__main__":
    main()
