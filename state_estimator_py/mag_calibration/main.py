import threading
import rclpy
import numpy as np
import plotly.graph_objects as go
from nicegui import ui, app

from data_controller import CalibrationController

# --- App State & Controller ---
rclpy.init()
controller = CalibrationController()
ui_state = {'show_calibrated': False}

# --- Plot Setup Helper (Plotly) ---
def create_mag_plot(title):
    fig = go.Figure()
    # Add an empty scatter trace
    fig.add_trace(go.Scatter(x=[], y=[], mode='markers', marker=dict(size=4, color='blue', opacity=0.5)))
    
    # Configure layout: equal aspect ratio is CRITICAL for ellipsoid calibration
    fig.update_layout(
        title=title,
        margin=dict(l=20, r=20, t=40, b=20),
        width=350, height=350,
        showlegend=False,
        yaxis=dict(scaleanchor="x", scaleratio=1), # Equivalent to plt.axis('equal')
        xaxis=dict(autorange=True),
    )
    return ui.plotly(fig)

# --- UI Action Handlers ---
def on_start():
    controller.start_acquisition()
    ui.notify('Acquisition Started', color='positive')

def on_stop():
    count = controller.stop_acquisition()
    ui.notify(f'Acquisition Stopped. Collected {count} points.', color='warning')

def on_clear():
    controller.clear_data()
    ui_state['show_calibrated'] = False
    output_textbox.set_value('')
    update_plots() # Visually clear the plots
    ui.notify('Data Cleared', color='info')

def on_calibrate():
    try:
        bias, matrix = controller.perform_calibration()
        output_str = (
            "import numpy as np\n\n"
            f"hard_iron_bias = np.array({np.array2string(bias, separator=', ')})\n\n"
            f"soft_iron_matrix = np.array({np.array2string(matrix, separator=', ')})"
        )
        output_textbox.set_value(output_str)
        ui.notify('Calibration Successful', color='positive')
        update_plots()
    except Exception as e:
        ui.notify(str(e), color='negative')

def on_toggle_view():
    ui_state['show_calibrated'] = not ui_state['show_calibrated']
    btn_text = "Revert to Raw" if ui_state['show_calibrated'] else "Apply Calibration (Preview)"
    toggle_btn.set_text(btn_text)
    update_plots()

# --- Plotting Subroutine ---
def update_plots():
    data = controller.get_plot_data(apply_cal=ui_state['show_calibrated'])
    
    # Handle empty data to ensure graphs clear properly
    if data is None or len(data) == 0:
        x, y, z = [], [], []
    else:
        x, y, z = data[:, 0], data[:, 1], data[:, 2]
        
    color = 'green' if ui_state['show_calibrated'] else 'blue'

    # Update XY Plot
    xy_plot.figure.data[0].x = x
    xy_plot.figure.data[0].y = y
    xy_plot.figure.data[0].marker.color = color
    xy_plot.update()

    # Update XZ Plot
    xz_plot.figure.data[0].x = x
    xz_plot.figure.data[0].y = z
    xz_plot.figure.data[0].marker.color = color
    xz_plot.update()

    # Update YZ Plot
    yz_plot.figure.data[0].x = y
    yz_plot.figure.data[0].y = z
    yz_plot.figure.data[0].marker.color = color
    yz_plot.update()

# --- UI Layout ---
ui.label('Magnetometer Calibration').classes('text-2xl font-bold mb-4')

with ui.row().classes('w-full gap-4 items-center mb-4'):
    ui.button('Start', on_click=on_start).classes('bg-green-600')
    ui.button('Stop', on_click=on_stop).classes('bg-red-600')
    ui.button('Clear Data', on_click=on_clear)
    ui.button('Calibrate', on_click=on_calibrate).classes('bg-blue-600')
    toggle_btn = ui.button('Apply Calibration (Preview)', on_click=on_toggle_view)

with ui.row().classes('w-full justify-center'):
    xy_plot = create_mag_plot("X vs Y")
    xz_plot = create_mag_plot("X vs Z")
    yz_plot = create_mag_plot("Y vs Z")

ui.label('Calibration Matrices (Copy/Paste):').classes('text-lg font-bold mt-4')
output_textbox = ui.textarea().classes('w-full font-mono').props('readonly rows=8')

# Update graphs at 2Hz
ui.timer(0.5, update_plots)

# --- ROS Threading & Lifecycle ---
def ros_spin_thread():
    rclpy.spin(controller)

thread = threading.Thread(target=ros_spin_thread, daemon=True)
thread.start()

app.on_shutdown(lambda: rclpy.shutdown())

ui.run(title="Mag Calibrator", port=8080)