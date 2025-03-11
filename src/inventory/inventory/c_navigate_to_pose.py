import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math



def compute_quaternion_from_yaw(yaw):
    """Compute quaternion from yaw angle (in radians)."""
    z = math.sin(yaw / 2.0)
    w = math.cos(yaw / 2.0)
    return z, w


def main():
    rclpy.init()

    # Create a node
    node = rclpy.create_node('waypoint_navigator')

    # Create an instance of the navigator
    navigator = BasicNavigator()

    # Set initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.10087973146222022
    initial_pose.pose.position.y = -0.050825397804589174
    z, w = compute_quaternion_from_yaw(0.0)
    initial_pose.pose.orientation.z = z
    initial_pose.pose.orientation.w = w
    navigator.setInitialPose(initial_pose)

    # Ensure navigation is active
    navigator.waitUntilNav2Active()
    
    # yaw angles 
    # right = -math.pi / 2
    # down = math.pi
    # left = math.pi / 2
    # up = 0.0

    # Define waypoints
    poses = [
        {"label": "p2", "x": 1.764, "y": -0.118, "yaw": -math.pi / 2},
        {"label": "p3", "x": 1.781, "y": -0.950, "yaw": math.pi},
        {"label": "p4", "x": -0.113, "y": -0.989, "yaw": math.pi / 2},
        {"label": "p1", "x": -0.100, "y": 0.050, "yaw": 0.0},
    ]

    # Navigate through waypoints
    for pose in poses:
        z, w = compute_quaternion_from_yaw(pose["yaw"])
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose["x"]
        goal_pose.pose.position.y = pose["y"]
        goal_pose.pose.orientation.z = z
        goal_pose.pose.orientation.w = w

        print(f"Navigating to {pose['label']} at x = {pose['x']}, y = {pose['y']}")
        navigator.goToPose(goal_pose)

        # Wait until the task is completed
        while not navigator.isTaskComplete():
            rclpy.spin_once(node, timeout_sec=0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"Successfully reached {pose['label']}.")
            
            
        else:
            print(f"Failed to reach {pose['label']}. Skipping to next waypoint.")

    print("Completed navigating all waypoints.")

    # Shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
