from pymongo import MongoClient
from datetime import datetime

# Connect to MongoDB
client = MongoClient("mongodb://localhost:27017/")
db = client["warehouse_inventory"]
storage_config = db["storage_config"]
metadata_collection = db["tag_metadata"]

# Define total storage units
total_storage_units = 9  # Change this number as needed

# Insert storage configuration (or update if it already exists)
storage_config.update_one({}, {"$set": {"total_units": total_storage_units}}, upsert=True)

# Define metadata for each known AprilTag with empty location fields and shelf set to None

metadata_entries = [
    
    #known items
    {
        "tag_id": "0",
        "name": "Electronics Box",
        "category": "Electronics",
        "weight": "2kg",
        "expiry": "N/A",
        "shelf": None,
        "location": None,
        "timestamp": datetime.now()
    },
    {
        "tag_id": "1",
        "name": "Fresh Apples Crate",
        "category": "Food",
        "weight": "5kg",
        "expiry": "2025-06-01",
        "shelf": None,
        "location": None,
        "timestamp": datetime.now()
    },
    {
        "tag_id": "2",
        "name": "Clothing Package",
        "category": "Clothing",
        "weight": "3kg",
        "expiry": "N/A",
        "shelf": None,
        "location": None,
        "timestamp": datetime.now()
    },

    # 5 and 6 will be in the inventory but will not be prenst in the enviroment 
    {
        "tag_id": "5",
        "name": "Sanitizer Pack",
        "category": "Pharmaceuticals",
        "weight": "1.2kg",
        "expiry": "2025-12-31",
        "shelf": None,
        "location": None,
        "timestamp": datetime.now()
    },
    {
        "tag_id": "6",
        "name": "Medical Supply",
        "category": "Pharmaceuticals",
        "weight": "1kg",
        "expiry": "2026-08-15",
        "shelf": None,
        "location": None,
        "timestamp": datetime.now()
    },
    
    # any other tag detected is assumed to be unknown 
    
]


# Insert into MongoDB
metadata_collection.insert_many(metadata_entries)
print("✅ Metadata with shelf=None and timestamp stored successfully in MongoDB.")
print(f"✅ Total storage units ({total_storage_units}) stored successfully in MongoDB.")
