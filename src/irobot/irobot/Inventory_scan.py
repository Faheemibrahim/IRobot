#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import math

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import subprocess
import uuid
from datetime import datetime
from pymongo import MongoClient
import time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def compute_quaternion_from_yaw(yaw):
    """Compute quaternion from yaw angle (radians)"""
    return math.sin(yaw/2), math.cos(yaw/2)


def run_capture_node(waypoint_label='A3'):
    class CaptureNode(Node):
        def __init__(self, label):
            super().__init__('capture_node')
            self.bridge = CvBridge()
            self.latest_pose = {"x": 0.0, "y": 0.0, "orientation": 0.0}
            self.pose_received = False
            self.waypoint_label = label
            self.tag_detected = None

            # MongoDB setup
            client = MongoClient("mongodb://localhost:27017/")
            db = client["warehouse_inventory"]
            self.metadata_collection = db["tag_metadata"]
            self.scan_collection = db["inventory_scans"]
            self.status_collection = db["inventory_status"]
            storage_config = db["storage_config"]
            self.storage_status_collection = db["storage_status"]
            storage_data = storage_config.find_one({})
            self.total_units = storage_data.get("total_units", 9) if storage_data else 9

            # QoS for amcl_pose (latch)
            pose_qos = QoSProfile(depth=10)
            pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            pose_qos.reliability = ReliabilityPolicy.RELIABLE

            # Subscriptions
            self.create_subscription(
                PoseWithCovarianceStamped,
                '/amcl_pose',
                self.pose_callback,
                qos_profile=pose_qos
            )
            # image topic
            img_qos = QoSProfile(depth=10)
            img_qos.reliability = ReliabilityPolicy.BEST_EFFORT
            self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                qos_profile=img_qos
            )

            self.get_logger().info(f"📷 Capture node initialized for shelf {label}")

        def pose_callback(self, msg: PoseWithCovarianceStamped):
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            siny = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy = 1 - 2 * (ori.y**2 + ori.z**2)
            theta = math.atan2(siny, cosy)
            self.latest_pose = {
                "x": round(pos.x, 2),
                "y": round(pos.y, 2),
                "orientation": round(math.degrees(theta), 2)
            }
            if not self.pose_received:
                self.get_logger().info(f"🗺️ Pose received: {self.latest_pose}")
            self.pose_received = True

        def image_callback(self, msg: Image):
            if not self.pose_received:
                self.get_logger().warn("⚠️ No pose yet, skipping AprilTag detection")
                return

            # capture and detect tag once
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_file = f'/tmp/{uuid.uuid4()}.png'
            cv2.imwrite(img_file, cv_image)

            try:
                result = subprocess.run([
                    '/home/faheem/anaconda3/envs/faheem/bin/python',
                    '/home/faheem/IRobot/src/inventory/subprocess/detect_apriltag.py',
                    img_file
                ], capture_output=True, text=True, timeout=5)
                out = result.stdout.strip()
                if "Detected Tag ID:" in out:
                    tag_id = out.split("Detected Tag ID:")[-1].strip()
                    self.get_logger().info(f"🎯 Detected Tag: {tag_id}")
                    self.process_detected_tag(tag_id, img_file)
                    self.tag_detected = tag_id
                else:
                    self.get_logger().info("🔍 No tag found in image.")
            except subprocess.TimeoutExpired:
                self.get_logger().error("⏱️ AprilTag detection timed out")
            finally:
                # shutdown after one detection
                self.destroy_node()

        def process_detected_tag(self, tag_id, img_path):
            now = datetime.now()
            loc = self.latest_pose
            shelf = self.waypoint_label
            
            # Check if tag exists already
            existing_tag = self.metadata_collection.find_one({"tag_id": tag_id})
            known = existing_tag is not None
            
            # Flag to track if this is an update (moved or new)
            is_update = False
            
            # Record scan
            self.scan_collection.insert_one({
                "tag_id": tag_id,
                "scanned_on": now,
                "location": loc,
                "image_path": img_path,
                "known": known,
                "shelf": shelf
            })
            
            # Update metadata based on whether it's known or new
            if known:
                # Check if location exists and if the shelf changed
                first = 'location' not in existing_tag
                moved = existing_tag.get("shelf") != shelf
                
                if first or moved:
                    # This is an update - tag moved or first time placing
                    is_update = True
                    self.get_logger().info(f"Updating tag {tag_id} location to {shelf}")
                    self.metadata_collection.update_one(
                        {"tag_id": tag_id},
                        {"$set": {
                            "location": loc, 
                            "shelf": shelf, 
                            "occupied": True,
                            "last_updated": now,  # Add timestamp of update
                            "is_update": True     # Flag as an update
                        },
                        "$inc": {"update_count": 1}}
                    )
                else:
                    # Tag is in the same place, just update the timestamp
                    self.metadata_collection.update_one(
                        {"tag_id": tag_id},
                        {"$set": {"last_scanned": now, "is_update": False}}
                    )
            else:
                # New unknown tag - definitely an update
                is_update = True
                self.get_logger().info(f"Creating new unknown tag {tag_id} at {shelf}")
                self.metadata_collection.insert_one({
                    "tag_id": tag_id,
                    "location": loc, 
                    "shelf": shelf, 
                    "occupied": True,
                    "category": "Unknown",
                    "last_updated": now,
                    "update_count": 1,
                    "is_update": True
                })
            
            # Make sure to call both update methods
            self.update_inventory_status()
            self.update_storage_status(is_update, shelf)

        def update_inventory_status(self):
            """Replaces the old update_missing_items method"""
            expected = {d["tag_id"] for d in self.metadata_collection.find({"category": {"$ne": "Unknown"}})}
            scanned = {d["tag_id"] for d in self.scan_collection.find({"known": True})}
            missing = expected - scanned
            unknown_count = self.scan_collection.count_documents({"known": False})

            self.status_collection.insert_one({
                "timestamp": datetime.now(),
                "missing_units": len(missing),
                "unknown_units": unknown_count,
                "updated_units": self.metadata_collection.count_documents({"is_update": True})
            })
            
            self.get_logger().info(f"Inventory status: {len(missing)} missing, {unknown_count} unknown")

        def update_storage_status(self, is_current_update=False, current_shelf=None):
            # Find all items with a valid shelf assignment and occupied status
            occupied_items = list(self.metadata_collection.find(
                {"occupied": True, "shelf": {"$exists": True}}
            ))
            
            # Use shelf as the identifier for occupied units
            occupied_shelves = [item["shelf"] for item in occupied_items if "shelf" in item]
            
            # Find updated units - only those marked with is_update:true
            updated_items = list(self.metadata_collection.find(
                {"is_update": True, "shelf": {"$exists": True}}
            ))
            updated_shelves = [item["shelf"] for item in updated_items if "shelf" in item]
            
            # Remove duplicates
            occupied_shelves = list(set(occupied_shelves))
            updated_shelves = list(set(updated_shelves))
            
            # If this scan resulted in an update, make sure it's in the updated list
            if is_current_update and current_shelf and current_shelf not in updated_shelves:
                updated_shelves.append(current_shelf)
            
            # Calculate empty units
            empty = max(self.total_units - len(occupied_shelves), 0)
            
            self.get_logger().info(f"Storage status: {len(occupied_shelves)} occupied, {len(updated_shelves)} updated")
            
            self.storage_status_collection.insert_one({
                "timestamp": datetime.now(),
                "total_units": self.total_units,
                "occupied_units": occupied_shelves,
                "updated_units": updated_shelves,  # Only include shelves with updates
                "total_scanned": self.scan_collection.count_documents({}),
                "empty_units": empty
            })

    # spin node until tag_detected or timeout
    capture = CaptureNode(waypoint_label)
    start = time.time()
    while rclpy.ok() and not capture.tag_detected and (time.time() - start) < 6:
        rclpy.spin_once(capture, timeout_sec=0.1)
    return capture.tag_detected


def main():
    rclpy.init()
    navigator = BasicNavigator()
    ctrl = rclpy.create_node('robot_controller')

    # initial pose
    init = PoseStamped()
    init.header.frame_id = 'map'
    init.header.stamp = navigator.get_clock().now().to_msg()
    init.pose.position.x = 0.0
    init.pose.position.y = 0.0
    iz, iw = compute_quaternion_from_yaw(0.0)
    init.pose.orientation.z = iz
    init.pose.orientation.w = iw
    navigator.setInitialPose(init)
    navigator.waitUntilNav2Active()

    waypoints = [
        {"label": "A3", "x": 0.0,  "y": 0.0,  "yaw": 0.0},
        {"label": "A2", "x": 1.03, "y": 0.0,  "yaw": 0.0},
        {"label": "A1", "x": 1.85, "y": 0.0,  "yaw": 0.0},
        {"label": "s",  "x": 2.2,  "y": -1.3, "yaw": math.pi},
        {"label": "C1", "x": 1.25, "y": -1.3, "yaw": math.pi},
        {"label": "C2", "x": 0.51, "y": -1.3, "yaw": math.pi}, 
        {"label": "C3", "x": -0.2,"y": -1.3, "yaw": math.pi}, # down
        {"label": "B3", "x": -0.10, "y": -1.3, "yaw": 0.0}, # up
        {"label": "B2", "x": 0.98, "y": -1.3, "yaw": 0.0},
        {"label": "B1", "x": 1.75, "y": -1.3, "yaw": 0.0},
    ]
    idx = 0
    while rclpy.ok() and idx < len(waypoints):
        wp = waypoints[idx]
        z, w = compute_quaternion_from_yaw(wp["yaw"])
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = navigator.get_clock().now().to_msg()
        goal.pose.position.x = wp["x"]
        goal.pose.position.y = wp["y"]
        goal.pose.orientation.z = z
        goal.pose.orientation.w = w

        print(f"Navigating to {wp['label']} at ({wp['x']}, {wp['y']})")
        navigator.goToPose(goal)
        while not navigator.isTaskComplete():
            rclpy.spin_once(ctrl, timeout_sec=0.1)

        if navigator.getResult() == TaskResult.SUCCEEDED:
            print(f"✅ Reached {wp['label']}")
            tag = run_capture_node(wp['label'])
            if tag:
                print(f"🎯 Tag ID {tag} detected at {wp['label']}")
            else:
                print("🔍 No tag detected.")
        else:
            print(f"❌ Failed to reach {wp['label']}")
        idx += 1

    print("Navigation completed.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()