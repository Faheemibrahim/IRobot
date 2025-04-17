#!/usr/bin/env python3
import socket
import ast
import math
from threading import Thread
from taipy.gui import Gui, State, get_state_id, invoke_callback
import taipy.gui.builder as tgb
from datetime import datetime
import pandas as pd
from pymongo import MongoClient
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
import tf_transformations

HOST = "127.0.0.1"
PORT = 5050
state_id_list = []

total_units = 0
occupied_units = 0
total_scanned = 0
updated_units = 0
unknown_units = 0
missing_units = 0

# Define waypoints for navigation
waypoints = [
    {"label": "A3", "x": 0.0,  "y": 0.0,  "yaw": 0.0},
    {"label": "A2", "x": 1.03, "y": 0.0,  "yaw": 0.0},
    {"label": "A1", "x": 1.85, "y": 0.0,  "yaw": 0.0},
    {"label": "C1", "x": 1.25, "y": -1.3, "yaw": math.pi},
    {"label": "C2", "x": 0.51, "y": -1.3, "yaw": math.pi},
    {"label": "C3", "x": -0.2, "y": -1.3, "yaw": math.pi},
    {"label": "B3", "x": -0.10, "y": -1.3, "yaw": 0.0},
    {"label": "B2", "x": 0.98, "y": -1.3, "yaw": 0.0},
    {"label": "B1", "x": 1.75, "y": -1.3, "yaw": 0.0},
    {"label": "Base", "x": 0.0, "y": -2.0, "yaw": 0.0},  # Added base position
]

# Navigation Status
current_navigation_status = "Idle"
current_target = None

class NavigationClient(Node):
    def __init__(self):
        super().__init__('navigation_client')
        self.navigate_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Navigation client initialized")
        self.current_goal_handle = None
        
    def send_goal(self, x, y, yaw):
        # Wait until action server is available
        self.get_logger().info(f"Waiting for navigation action server...")
        self.navigate_to_pose_client.wait_for_server()
        
        # Create goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set the pose
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]
        
        self.get_logger().info(f"Sending goal: x={x}, y={y}, yaw={yaw}")
        
        # Send the goal
        send_goal_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # Add done callback
        send_goal_future.add_done_callback(self.goal_response_callback)
        return send_goal_future
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.current_goal_handle = goal_handle
        
        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Process feedback if needed
        pass
    
    def cancel_navigation(self):
        if self.current_goal_handle is not None:
            self.get_logger().info("Cancelling current navigation goal")
            cancel_future = self.current_goal_handle.cancel_goal_async()
            return cancel_future
        return None

def fetch_data():
    try:
        client = MongoClient("mongodb://localhost:27017/")
        db = client["warehouse_inventory"]
        collection = db["tag_metadata"]
        data_list = list(collection.find())

        for doc in data_list:
            doc.pop("_id", None)

        df = pd.DataFrame(data_list)

        if not df.empty and "location" in df.columns:
            df["location"] = df["location"].apply(
                lambda loc: f"x: {loc.get('x')}, y: {loc.get('y')}, orientation: {loc.get('orientation')}"
                if isinstance(loc, dict) else loc
            )

        client.close()
        return df
    except Exception as e:
        print("❌ Error fetching data:", e)
        return pd.DataFrame()

# ROS2 initialization
def init_ros():
    global nav_client
    rclpy.init(args=None)
    nav_client = NavigationClient()
    print("✅ ROS2 and Nav2 client initialized")
    return nav_client

# Thread for ROS2 spinning
def ros_spin(node):
    rclpy.spin(node)

# Initial values
data = fetch_data()
last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
nav_client = None

def refresh_table(state: State):
    state.data = fetch_data()
    state.last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print("🔄 Table refreshed.")

def on_init(state: State):
    """Initialize the app state with the required variables."""
    global nav_client
    
    # Initialize ROS2 and Nav2 client
    nav_client = init_ros()
    
    # Start ROS2 spin thread
    ros_thread = Thread(target=ros_spin, args=(nav_client,))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Initialize navigation status
    state.current_navigation_status = "Idle"
    state.current_target = None
    
    state_id = get_state_id(state)
    if state_id:
        state_id_list.append(state_id)

def client_handler(gui: Gui, state_id_list: list):
    """Handles receiving data from sender.py."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind((HOST, PORT))
    s.listen()
    print(f"🔌 Listening on {HOST}:{PORT} …")
    conn, _ = s.accept()
    
    while True:
        data = conn.recv(1024)
        if data:
            try:
                # Convert received string back to a dictionary.
                received_dict = ast.literal_eval(data.decode())
                print(f"📥 Received Data: {received_dict}")
                if state_id_list:
                    invoke_callback(gui, state_id_list[0], update_received_data, (received_dict,))
            except Exception as e:
                print(f"⚠️ Error processing received data: {e}")
        else:
            print("🔌 Connection closed")
            break

def update_received_data(state: State, received_dict):
    """Updates the GUI state with received inventory data."""
    state.total_units = received_dict["total_units"]
    state.occupied_units = received_dict["occupied_units"]
    state.total_scanned = received_dict["total_scanned"]
    state.updated_units = received_dict["updated_units"]
    state.unknown_units = received_dict["unknown_units"]
    state.missing_units = received_dict["missing_units"]

def navigate_to_waypoint(state: State, id, payload):
    """Function that navigates the robot to a waypoint when a button is clicked."""
    global nav_client
    
    # First cancel any ongoing navigation
    if state.current_navigation_status != "Idle":
        nav_client.cancel_navigation()
    
    # Find the waypoint that matches the button id
    waypoint = next((wp for wp in waypoints if wp["label"] == id), None)
    if waypoint:
        print(f"🚀 Navigating to {id} at x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']}")
        state.current_navigation_status = f"Navigating to {id}"
        state.current_target = id
        
        # Send navigation goal
        nav_client.send_goal(waypoint["x"], waypoint["y"], waypoint["yaw"])
    else:
        print(f"⚠️ Waypoint {id} not found")

def cancel_navigation(state: State, id, payload):
    """Cancel current navigation goal."""
    global nav_client
    
    if state.current_navigation_status != "Idle":
        nav_client.cancel_navigation()
        print("🛑 Navigation cancelled")
        state.current_navigation_status = "Idle"
        state.current_target = None
    
with tgb.Page() as page:
    # Panel 1: Main container with header and three cards
    with tgb.part(class_name="container"):
        tgb.text("# 📦 IRobot Dashboard", class_name="title", mode="md")
        
        with tgb.layout(columns="1 1 1 1", gap="5px"):
            with tgb.part(class_name="card"):
                tgb.text(value="#### 📸 Total scanned: {total_scanned}", mode="md", class_name="centered-text")
            with tgb.part(class_name="card"):
                tgb.text(value="#### ⚠️ Missing units: {missing_units}", mode="md", class_name="centered-text")
            with tgb.part(class_name="card"):
                tgb.text(value="#### ❓ Unknown units: {unknown_units}", mode="md", class_name="centered-text")
            with tgb.part(class_name="card"):
                tgb.text(value="#### 🔄 Updated units: {updated_units}", mode="md", class_name="centered-text")
    
    tgb.html("br")
    with tgb.part(class_name="container", height="300px"):
        with tgb.layout(columns="1 1", gap="10px"):
            with tgb.part(class_name="card"):
                tgb.text("# 🖥️ Robot control", mode="md")
                # Status indicator
                tgb.text("#### 🚦 Status: {current_navigation_status}", mode="md")
                if "{current_target}":
                    tgb.text("#### 🎯 Target: {current_target}", mode="md")
                
                # Grid of buttons in a 5-column layout
                with tgb.layout(columns="1 1 1 1 1", gap="5px"):
                    tgb.button("Base", id="Base", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("A1", id="A1", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("A2", id="A2", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("A3", id="A3", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("B1", id="B1", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("B2", id="B2", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("B3", id="B3", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("C1", id="C1", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("C2", id="C2", on_action=navigate_to_waypoint, width="100%", height="100%")
                    tgb.button("C3", id="C3", on_action=navigate_to_waypoint, width="100%", height="100%")
                
                # Add cancel button
                tgb.button("Cancel Navigation", id="cancel", on_action=cancel_navigation, width="100%", height="50px")
                
            with tgb.part(class_name="card"):
                tgb.text("# 📊 Occupancy & Availability", mode="md")
                tgb.metric("{occupied_units}", format="%.1i/9", min=0, max=9)
        tgb.html("br")
        tgb.table("{data}")
        tgb.html("br")
        tgb.button("Refresh", id="Refresh", on_action=refresh_table, width="100%", height="100%")

# Initial values
data = fetch_data()
last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
current_navigation_status = "Idle"
current_target = None

if __name__ == "__main__":
    gui = Gui(page=page)
    receiver_thread = Thread(target=client_handler, args=(gui, state_id_list))
    receiver_thread.daemon = True
    receiver_thread.start()
    
    try:
        gui.run(title="Warehouse Inventory Dashboard", on_init=on_init)
    finally:
        # Clean up ROS2 resources when application closes
        if nav_client:
            nav_client.destroy_node()
        rclpy.shutdown()