from taipy.gui import Gui, State, navigate
import taipy.gui.builder as tgb
from taipy.gui.icon import Icon

# Default values
total_units = 9
occupied_units = 3

total_scanned = 0
updated_units = 0
unknown_units = 0
missing_units = 0

# Function to handle menu selection and navigate to the selected page
def menu_option_selected(state: State, action, info):
    page = info["args"][0]  # Extracts the selected page
    navigate(state, to=page)  # Navigates to the selected page


def on_button_click(state: State, id, payload):
    """Function that prints a message when a button is clicked."""
    print(f"🖱️ Button {id} clicked! Printing {id} on the command line.")

# Define the Root Page (Main Menu)
with tgb.Page() as root_page:
    tgb.menu(
        label="Menu",
        lov=[
            ("page1", Icon("images/map.png"), "Sales"), 
            ("page2", Icon("images/person.png"), "Account"),
            ("page3", Icon("images/person.png"), "Account2")
            
        ],
        on_action=menu_option_selected
    )

# Define Page 1 (Sales Page)
with tgb.Page() as page_1:
    # Panel 1: Main container with header and three cards
    with tgb.part(class_name="container"):
        tgb.text("# 📦 IRobot", class_name="title", mode="md")
        
        with tgb.layout(columns="1 1 1 1", gap="5px"):
            with tgb.part(class_name="card"):
                tgb.text(value="#### 📸 Total scanned: {total_scanned}", mode="md", class_name="centered-text")
            with tgb.part(class_name="card"):
                tgb.text(value="#### ⚠️ Missing units: {missing_units}", mode="md", class_name="centered-text")
            with tgb.part(class_name="card"):
                tgb.text(value="#### ❓ Unknown units: {unknown_units}", mode="md", class_name="centered-text")
            with tgb.part(class_name="card"):
                tgb.text(value="#### 🔄 Updated units: {updated_units}", mode="md", class_name="centered-text")

            
    # Panel 2: Additional metrics panel positioned below Panel 1, now made smaller.
    # Adding a height parameter (e.g., "300px") to make it smaller.
    with tgb.part(class_name="container", height="300px"):
        tgb.text("## Additional Metrics", class_name="subtitle", mode="md")
        
        with tgb.layout(columns="1 1", gap="10px"):
            with tgb.part(class_name="card"):
                tgb.text("# 🖥️ Robot control", mode="md")
                # Grid of buttons in a 5-column layout
                with tgb.layout(columns="1 1 1 1 1", gap="5px"):
                    tgb.button("Base", id="Base", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P1", id="p1", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P2", id="p2", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P3", id="p3", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P4", id="p4", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P5", id="p5", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P6", id="p6", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P7", id="p7", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P8", id="p8", on_action=on_button_click,width="100%", height="100%")
                    tgb.button("P9", id="p9", on_action=on_button_click,width="100%", height="100%")
            with tgb.part(class_name="card"):
                tgb.text("# 📊 Occupancy & Availability", mode="md")
                tgb.metric("{occupied_units}", format="%.1i/{total_units}", min=0, max=10)


# Define Page 2 (Account Page)
with tgb.Page() as page_2:
    tgb.text("# Account **Management**", mode="md")
    tgb.button("Logout", class_name="plain login-button", width="50px")

with tgb.Page() as page_3:
    tgb.text("# Account **Management**", mode="md")
    tgb.button("Logout", class_name="plain login-button", width="50px")

# Define the Page Mapping
pages = {
    "/": root_page,
    "page1": page_1,
    "page2": page_2,
    "page3": page_3
    
}

# Run the Web App
gui = Gui(pages=pages)
gui.run(title="Multi-Page App", port=5000)
