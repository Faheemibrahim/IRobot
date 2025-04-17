#!/usr/bin/env python3
"""
Capture Node Script:
- Detects AprilTags using ROS 2 and OpenCV.
- Logs scans in the inventory_scans collection.
- Updates metadata in the tag_metadata collection.
- For unknown tags (i.e. not found in metadata), it stores them with category "Unknown".
- Updates and logs overall inventory metrics in the storage_status and inventory_status collections.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess
import sys
import uuid
from datetime import datetime
from pymongo import MongoClient

class CaptureNode(Node):
    def __init__(self):
        super().__init__('capture_node')
        self.bridge = CvBridge()

        # MongoDB Connection
        self.client = MongoClient("mongodb://localhost:27017/")
        self.db = self.client["warehouse_inventory"]
        self.metadata_collection = self.db["tag_metadata"]
        self.scan_collection = self.db["inventory_scans"]
        self.status_collection = self.db["inventory_status"]
        self.storage_config = self.db["storage_config"]
        self.storage_status_collection = self.db["storage_status"]

        # Track updates during this run
        self.update_counter = 0

        # Fetch total units (default to 50)
        storage_data = self.storage_config.find_one({})
        self.total_units = storage_data["total_units"] if storage_data else 50

        # AprilTag detection setup
        self.conda_python = '/home/faheem/anaconda3/envs/faheem/bin/python'
        self.detect_script = '/home/faheem/IRobot/src/inventory/subprocess/detect_apriltag.py'

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.get_logger().info("AprilTag Capture Node Ready")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            temp_filename = f'/tmp/{uuid.uuid4()}.png'
            cv2.imwrite(temp_filename, cv_image)

            result = subprocess.run(
                [self.conda_python, self.detect_script, temp_filename],
                capture_output=True,
                text=True
            )
            detect_output = result.stdout.strip()

            if "Detected Tag ID:" in detect_output:
                detected_tag_id = detect_output.split("Detected Tag ID:")[-1].strip()
                self.get_logger().info(f"AprilTag detected: {detected_tag_id}")
                self.process_detected_tag(detected_tag_id, temp_filename)
            elif detect_output == "no tags found":
                self.get_logger().info("No AprilTags detected.")
                self.update_missing_items()
            else:
                self.get_logger().error(f"Unexpected response: {detect_output}")

            self.get_logger().info("Shutting down after detection.")
            rclpy.shutdown()
            sys.exit(0)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            rclpy.shutdown()
            sys.exit(1)

    def process_detected_tag(self, tag_id, image_path):
        scanned_on = datetime.now()
        # For testing, change actual_location to simulate movement.
        actual_location = {"x": 60, "y": 10, "orientation": 90}
        storage_unit = f"Unit_{int(actual_location['x'])}_{int(actual_location['y'])}"

        # Query metadata to check if the tag exists.
        expected_tag = self.metadata_collection.find_one({"tag_id": tag_id})
        is_known = expected_tag is not None

        # Log the scan event (applies to both known and unknown tags).
        self.scan_collection.insert_one({
            "tag_id": tag_id,
            "scanned_on": scanned_on,
            "location": actual_location,
            "image_path": image_path,
            "known": is_known
        })

        if is_known:
            # Process known tag update.
            location = expected_tag.get("location", {})
            if not location or None in location.values():
                self.metadata_collection.update_one(
                    {"tag_id": tag_id},
                    {"$set": {"location": actual_location, "update_count": 1, "occupied": True}}
                )
                self.update_counter += 1
                self.get_logger().info(f"📍 First-time update for Tag ID {tag_id}: Location set to {actual_location}")
            else:
                tolerance = 0.5
                if (abs(actual_location["x"] - location["x"]) > tolerance or
                    abs(actual_location["y"] - location["y"]) > tolerance):
                    self.metadata_collection.update_one(
                        {"tag_id": tag_id},
                        {"$set": {"location": actual_location, "occupied": True}, "$inc": {"update_count": 1}}
                    )
                    self.update_counter += 1
                    self.get_logger().info(f"📍 Item location updated for Tag ID {tag_id}: {actual_location}")
        else:
            # Tag is not in metadata: treat it as unknown.
            # Check if an unknown record for this tag already exists.
            unknown_record = self.metadata_collection.find_one({"tag_id": tag_id, "category": "Unknown"})
            if unknown_record:
                # Update if location differs (within a tolerance).
                location = unknown_record.get("location", {})
                tolerance = 0.5
                if not location or (abs(actual_location["x"] - location["x"]) > tolerance or
                                    abs(actual_location["y"] - location["y"]) > tolerance):
                    self.metadata_collection.update_one(
                        {"tag_id": tag_id, "category": "Unknown"},
                        {"$set": {"location": actual_location, "occupied": True}, "$inc": {"update_count": 1}}
                    )
                    self.update_counter += 1
                    self.get_logger().info(f"📍 Updated unknown tag record for Tag ID {tag_id}: {actual_location}")
            else:
                # Insert a new record for the unknown tag.
                self.metadata_collection.insert_one({
                    "tag_id": tag_id,
                    "category": "Unknown",
                    "location": actual_location,
                    "update_count": 1,
                    "occupied": True
                })
                self.update_counter += 1
                self.get_logger().info(f"🚨 New unknown item detected and recorded: Tag ID {tag_id} with location {actual_location}")

        # Always update missing/unknown metrics after processing.
        self.update_missing_items()

    def update_missing_items(self):
        # Only consider known tags (those with category not equal to "Unknown") as expected.
        expected_tags = {doc["tag_id"] for doc in self.metadata_collection.find({"category": {"$ne": "Unknown"}})}
        scanned_tags = {doc["tag_id"] for doc in self.scan_collection.find({"known": True})}
        unknown_tags = list(self.scan_collection.find({"known": False}))

        self.missing_units = list(expected_tags - scanned_tags)
        self.unknown_units = len(unknown_tags)

        # Insert/update the inventory_status record.
        self.status_collection.insert_one({
            "timestamp": datetime.now(),
            "missing_units": len(self.missing_units),
            "unknown_units": self.unknown_units,
            "updated_units": self.update_counter
        })

        self.get_logger().info(f"🚨 Missing Known Items: {len(self.missing_units)}")
        self.get_logger().info(f"❓ Unknown Items: {self.unknown_units}")

        # Proceed to update storage status metrics.
        self.update_storage_status()

    def update_storage_status(self):
        # Build occupied units from metadata (only those marked as occupied)
        occupied_units = {
            f"Unit_{doc['location']['x']}_{doc['location']['y']}"
            for doc in self.metadata_collection.find({"occupied": True, "location": {"$exists": True, "$ne": {}}})
        }
        occupied_count = len(occupied_units)
        empty_units = max(self.total_units - occupied_count, 0)

        # For missing, recalc expected known tags vs scanned tags.
        expected_tags = {doc["tag_id"] for doc in self.metadata_collection.find({"category": {"$ne": "Unknown"}})}
        scanned_tags = {doc["tag_id"] for doc in self.scan_collection.find({"known": True})}
        unknown_tags = list(self.scan_collection.find({"known": False}))
        missing_units = list(expected_tags - scanned_tags)

        total_scanned = self.scan_collection.count_documents({})

        self.storage_status_collection.insert_one({
            "timestamp": datetime.now(),
            "total_units": self.total_units,
            "occupied_units": list(occupied_units),
            "total_scanned": total_scanned,
            "updated_units": self.update_counter,
            "empty_units": empty_units
        })

        self.get_logger().info(f"\n🏠 Total Units: {self.total_units}")
        self.get_logger().info(f"✅ Occupied Units: {occupied_count}")
        self.get_logger().info(f"🆓 Empty Units: {empty_units}")
        self.get_logger().info(f"🚨 Missing units: {len(missing_units)}")
        self.get_logger().info(f"❓ Unknown units: {len(unknown_tags)}")
        self.get_logger().info(f"🔄 Updated Units (this run): {self.update_counter}")
        self.get_logger().info(f"🧾 Total Scanned units (all time): {total_scanned}")

def main(args=None):
    rclpy.init(args=args)
    node = CaptureNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
