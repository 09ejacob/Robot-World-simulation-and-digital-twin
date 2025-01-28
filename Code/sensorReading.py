import omni.ui as ui
import asyncio
from omni.isaac.sensor import _sensor

_contact_sensor_interface = _sensor.acquire_contact_sensor_interface()
sensor_prim_path = "/World/cube1/Contact_Sensor"

window = ui.Window("Contact Sensor Data Window", width=400, height=250)

with window.frame:
    with ui.VStack():
        title = ui.Label("Contact Sensor Data", style={"font_size": 15, "color": "white"})
        label_sensor = ui.Label("Force:")
        progress_sensor = ui.ProgressBar()
        value_label = ui.Label("0.000 N")


async def update_sensor_values():
    while True:
        sensor_data = _contact_sensor_interface.get_sensor_reading(sensor_prim_path, use_latest_data=True)
        
        if sensor_data:
            force_value = getattr(sensor_data, "value", 0.0)
            #print(f"Force Value: {force_value}") 

            normalized_force = min(force_value / 20, 1.0)

            progress_sensor.model.set_value(normalized_force)
            value_label.text = f"{force_value:.3f} N"

        await asyncio.sleep(0.1)


asyncio.ensure_future(update_sensor_values())