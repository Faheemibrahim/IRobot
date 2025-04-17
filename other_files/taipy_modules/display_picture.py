import os
from taipy.gui import Gui, State
import taipy.gui.builder as tgb


img_path = "/home/faheem/IRobot/src/inventory/detected_tags/0.png"

# Create the Taipy Web Page
with tgb.Page() as page:
            
    tgb.image(img_path, label=os.path.basename(img_path), id=img_path)

# Run the Web App
gui = Gui(page=page)
gui.run(title="Image Viewer", port=5000)
