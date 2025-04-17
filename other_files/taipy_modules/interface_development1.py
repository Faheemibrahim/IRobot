from taipy.gui import Gui, State
import taipy.gui.builder as tgb
import pandas as pd
from pymongo import MongoClient
from datetime import datetime
from taipy.gui import Gui, State
import taipy.gui.builder as tgb

# Default values
total_units = 9
occupied_units = 3

total_scanned = 0
updated_units = 0
unknown_units = 0
missing_units = 0

# Fetch data from MongoDB
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

# Initial values
data = fetch_data()
last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

# Initial values
data = fetch_data()
last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

def refresh_table(state: State):
    state.data = fetch_data()
    state.last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print("🔄 Table refreshed.")

def on_button_click(state: State, id, payload):
    """Function that prints a message when a button is clicked."""
    print(f"🖱️ Button {id} clicked! Printing {id} on the command line.")

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
                # Grid of buttons in a 5-column layout
                with tgb.layout(columns="1 1 1 1 1", gap="5px"):
                    tgb.button("Base", id="Base", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("A1", id="A1", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("A2", id="A2", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("A3", id="A3", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("B1", id="B1", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("B2", id="B2", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("B3", id="B3", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("C1", id="C1", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("C2", id="C2", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("C3", id="C3", on_action=on_button_click,width="100%", height="100%")
            with tgb.part(class_name="card"):
                tgb.text("# 📊 Occupancy & Availability", mode="md")
                tgb.metric("{occupied_units}", format="%.1i/{total_units}", min=0, max=10)
        tgb.html("br")
        tgb.table("{data}")
        tgb.html("br")
        tgb.button("Refresh", id="Refresh", on_action=refresh_table,width="100%", height="100%")
                    
# Initial values
data = fetch_data()
last_refresh = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

gui = Gui(page=page)
gui.run(title="Taipy Button Grid Example", port=5000)
