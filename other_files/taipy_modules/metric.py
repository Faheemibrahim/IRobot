import os
from taipy.gui import Gui, State
import taipy.gui.builder as tgb

value = 3
# Create the Taipy Web Page
with tgb.Page() as page:
            
    tgb.metric("{value}",format="%.1i/10", min=0, max=10)
    #tgb.metric("{co2_2024}", delta="{delta}", delta_color="invert", format="%.1f ppm", delta_format="%.1f ppm", min=300, max=500)
# Run the Web App
gui = Gui(page=page)
gui.run(title="Image Viewer", port=5000)
