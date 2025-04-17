#!/usr/bin/env python3
"""
Sender Script: Retrieves inventory data from MongoDB and sends it over a socket.
It retrieves data using these keys:
  - total_units
  - occupied_units
  - empty_units
  - total_scanned
  - updated_units
  - unknown_units
  - missing_units
and adds a timestamp.
The data is merged from:
  - storage_status collection (for total_units, occupied_units, empty_units, total_scanned, updated_units)
  - inventory_status collection (for unknown_units, missing_units)
This data is sent every 5 seconds.
"""

import time
import socket
from pymongo import MongoClient, errors
from datetime import datetime

HOST = "127.0.0.1"  # The receiver's address
PORT = 5050         # The receiver's port

def get_inventory_data_from_mongodb():
    """
    Connects to the 'warehouse_inventory' database and retrieves the latest metrics.
    Data is merged from:
      - storage_status (for: total_units, occupied_units, empty_units, total_scanned, updated_units)
      - inventory_status (for: unknown_units, missing_units)
    Returns a dictionary with the keys and a current timestamp.
    """
    try:
        client = MongoClient("mongodb://localhost:27017/", serverSelectionTimeoutMS=5000)
        db = client["warehouse_inventory"]

        # Retrieve total_units from storage_config as fallback
        storage_config = db["storage_config"].find_one() or {}
        total_units_config = storage_config.get("total_units", 0)

        # Retrieve the latest document from the storage_status collection.
        storage_status = db["storage_status"].find_one(sort=[("timestamp", -1)]) or {}
        # Use the stored total_units if available; otherwise fall back to storage_config.
        total_units = storage_status.get("total_units", total_units_config)
        occupied_units_list = storage_status.get("occupied_units", [])
        occupied_units = len(occupied_units_list)
        empty_units = storage_status.get("empty_units", max(total_units - occupied_units, 0))
        total_scanned = storage_status.get("total_scanned", 0)
        updated_units = storage_status.get("updated_units", 0)

        # Retrieve the latest document from the inventory_status collection.
        inventory_status = db["inventory_status"].find_one(sort=[("timestamp", -1)]) or {}
        missing_units = inventory_status.get("missing_units", 0)
        unknown_units = inventory_status.get("unknown_units", 0)

        client.close()

        return {
            "total_units": total_units,
            "occupied_units": occupied_units,
            "empty_units": empty_units,
            "total_scanned": total_scanned,
            "updated_units": updated_units,
            "missing_units": missing_units,
            "unknown_units": unknown_units,
            "retrieved_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }

    except errors.PyMongoError as e:
        print(f"❌ MongoDB Error: {e}")
        return None

def send_data():
    """Connects to the receiver on HOST:PORT and sends the inventory data every 5 seconds."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.connect((HOST, PORT))
            print(f"✅ Connected to Receiver at {HOST}:{PORT}")
            while True:
                inventory_data = get_inventory_data_from_mongodb()
                if inventory_data:
                    message = str(inventory_data)  # Convert the dictionary to a string.
                    s.sendall(message.encode())
                    print(f"📤 Sent inventory update: {message}")
                else:
                    print("⚠️ Skipping send - No data fetched.")
                time.sleep(5)
        except ConnectionRefusedError:
            print("❌ Receiver is not running. Start the receiver script first.")
        except Exception as e:
            print(f"⚠️ Unexpected Error: {e}")

if __name__ == "__main__":
    send_data()
