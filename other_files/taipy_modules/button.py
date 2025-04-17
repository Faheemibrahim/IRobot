from taipy.gui import Gui, State
import taipy.gui.builder as tgb

def on_button_click(state: State, id, payload):
    """Function that prints a message when a button is clicked."""
    print(f"🖱️ Button {id} clicked! Printing {id} on the command line.")

# Create the Web PageP
with tgb.Page() as page:
    with tgb.part():
            with tgb.layout(rows="1 1 1 1 1"):
                with tgb.layout(gap="1000px"):
                    with tgb.layout(columns="1 1 1 1 1"):
                        tgb.button("Base", id="Base", on_action=on_button_click)
                        tgb.button("P1", id="p1", on_action=on_button_click)
                        tgb.button("P2", id="p2", on_action=on_button_click)
                        tgb.button("P3", id="p3", on_action=on_button_click)
                        tgb.button("P4", id="p4", on_action=on_button_click)
                        tgb.button("P5", id="p5", on_action=on_button_click)
                        tgb.button("P6", id="p6", on_action=on_button_click)
                        tgb.button("P7", id="p7", on_action=on_button_click)
                        tgb.button("P8", id="p8", on_action=on_button_click)
                        tgb.button("P9", id="p9", on_action=on_button_click)

# Run the Web App
gui = Gui(page=page)
gui.run(title="Taipy Button Grid Example", port=5000)