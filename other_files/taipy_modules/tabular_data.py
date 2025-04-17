from taipy.gui import Gui, State, navigate, Markdown
import taipy.gui.builder as tgb
from taipy.gui.icon import Icon
import os
from datetime import datetime
import pandas as pd
import pandas as pd
from pymongo import MongoClient
from datetime import datetime

data = pd.DataFrame({
    "tag_id": ["0", "1", "2"],
    "name": ["Electronics Box", "Medical Supply", "Clothing Package"],
    "category": ["Electronics", "Pharmaceuticals", "Clothing"],
    "weight": ["2kg", "1kg", "3kg"],
    "expiry": ["N/A", "2026-08-15", "N/A"],
    "location_x": [None, None, None],
    "location_y": [None, None, None],
    "orientation": [None, None, None],
    "timestamp": [datetime.now(), datetime.now(), datetime.now()]
})

# Create the Taipy Web Page
with tgb.Page() as page:
    with tgb.part(class_name="container"):
        tgb.text("# Inventory Management", class_name="title", mode="md")
        tgb.html("br")
        tgb.table("{data}")
            
    
# Run the Web App
gui = Gui(page=page)
gui.run(title="Image Viewer", port=5000)
