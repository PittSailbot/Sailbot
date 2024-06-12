import tkinter as tk
from tkinter import ttk
import odrive
import time
import threading
from datetime import datetime

od = odrive.find_any()
axis = od.axis0
delay_time = 3


def move():
    target = 0
    start_time = datetime.now()
    while True:
        if (datetime.now() - start_time).total_seconds() > float(delay_time):
            target = (target + 1) % 2
            axis.controller.input_pos = target
            start_time = datetime.now()
        else:
            time.sleep(0.05)


def on_slider1_change(value):
    label1.config(text=f"pos_gain: {round(float(value), 2)}")
    axis.controller.config.pos_gain = value


def on_slider2_change(value):
    label2.config(text=f"vel_gain: {round(float(value), 2)}")
    axis.controller.config.vel_gain = value


def on_slider3_change(value):
    label3.config(text=f"vel_integrator_gain: {round(float(value), 2)}")
    axis.controller.config.vel_integrator_gain = value


def on_slider4_change(value):
    label4.config(text=f"Current Limit: {round(float(value), 2)}")
    axis.motor.config.current_lim = value


def on_slider5_change(value):
    label5.config(text=f"Max Vel: {round(float(value), 2)}")
    axis.controller.config.vel_limit = value


def on_slider6_change(value):
    global delay_time
    label6.config(text=f"rotate delay: {round(float(value), 2)}")
    delay_time = value


# Create the main window
root = tk.Tk()
root.title("Odrive tune")
slider_width = 300

# Create Slider 1
label1 = ttk.Label(root, text="pos_gain")
label1.pack(pady=10)
slider1 = ttk.Scale(
    root,
    from_=0,
    to_=150,
    command=on_slider1_change,
    orient='horizontal',
    length=slider_width,
)
slider1.set(axis.controller.config.pos_gain)
slider1.pack(pady=10)

# Create Slider 2
label2 = ttk.Label(root, text="vel_gain")
label2.pack(pady=10)
slider2 = ttk.Scale(root, from_=0, to_=10, command=on_slider2_change, orient='horizontal', length=slider_width)
slider2.set(axis.controller.config.vel_gain)
slider2.pack(pady=10)

# Create Slider 3
label3 = ttk.Label(root, text="vel_integrator_gain")
label3.pack(pady=10)
slider3 = ttk.Scale(root, from_=0, to_=100, command=on_slider3_change, orient='horizontal', length=slider_width)
slider3.set(axis.controller.config.vel_integrator_gain)
slider3.pack(pady=10)

# Create Slider 4
label4 = ttk.Label(root, text="Current Limit")
label4.pack(pady=10)
slider4 = ttk.Scale(root, from_=0, to_=50, command=on_slider4_change, orient='horizontal', length=slider_width)
slider4.set(axis.motor.config.current_lim)
slider4.pack(pady=10)

# Create Slider 5
label5 = ttk.Label(root, text="Velocity Limit")
label5.pack(pady=10)
slider5 = ttk.Scale(root, from_=0, to_=10, command=on_slider5_change, orient='horizontal', length=slider_width)
slider5.set(axis.controller.config.vel_limit)
slider5.pack(pady=10)

# Create Slider 6
label6 = ttk.Label(root, text="rotate delay")
label6.pack(pady=10)

slider6 = ttk.Scale(root, from_=0, to_=10, command=on_slider6_change, orient='horizontal', length=slider_width)
slider6.set(delay_time)
slider6.pack(pady=10)

threading.Thread(target=move).start()
# Start the main loop
root.mainloop()
