from tkinter import *
import tkinter.font as tkFont
import serial.tools.list_ports

ser = None # Serial port
root = Tk()
root.geometry("390x800")

def connect_to_arduino(port):
    global ser
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=0.01)
        print(f"Connected to {port}")
    except Exception as e:
        print(f"Error: {e}")
    
def refresh_ports():
    # Get available ports
    ports = list(serial.tools.list_ports.comports())
    return [port.device for port in ports]

def checkSerialPort():
    if ser is not None and ser.isOpen() and ser.in_waiting:
        recentPacket = ser.readline()
        recentPacketString = recentPacket.decode('utf-8').rstrip('\n')
        if ";" in recentPacketString:  # Check for delimiter
            parts = recentPacketString.split(";")
            x_value = parts[0].replace("X", "").strip()
            y_value = parts[1].replace("Y", "").strip()
            # Update label text variables
            x_var.set(f"X: {x_value}")
            y_var.set(f"Y: {y_value}")
        else:
            # Optionally handle data without delimiter
            print(recentPacketString)
            pass  # or clear labels, etc.

def on_connect():
    connect_to_arduino(selected_port.get())

def refresh_dropdown():
    global available_ports
    available_ports = refresh_ports()
    menu = dropdown["menu"]
    menu.delete(0, "end")
    for port in available_ports:
        menu.add_command(label=port, command=lambda v=port: selected_port.set(v))
    # Optionally reset to default after refresh:
    default_port = "/dev/cu.usbmodem1101"
    if default_port in available_ports:
        selected_port.set(default_port)
    elif available_ports:
        selected_port.set(available_ports[0])
    else:
        selected_port.set("None")
    # print("Port list refreshed.")

def left_press(event):
    # print("Left button pressed")
    if ser and ser.is_open:
        ser.write(b'L')

def left_release(event):
    # print("Left button released")
    if ser and ser.is_open:
        ser.write(b'S')

def right_press(event):
    # print("Right button pressed")
    if ser and ser.is_open:
        ser.write(b'R')

def right_release(event):
    # print("Right button released")
    if ser and ser.is_open:
        ser.write(b'S')

def up_press(event):
    # print("Up button pressed")
    if ser and ser.is_open:
        ser.write(b'U')

def up_release(event):
    # print("Up button released")
    if ser and ser.is_open:
        ser.write(b'S')

def down_press(event):
    # print("Down button pressed")
    if ser and ser.is_open:
        ser.write(b'D')

def down_release(event):
    # print("Down button released")
    if ser and ser.is_open:
        ser.write(b'S')

def disable_press():
    # print("Disable button pressed")
    if ser and ser.is_open:
        ser.write(b'F')
    disable_step_button.config(highlightbackground="green", relief="sunken")
    largest_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    middle_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    smallest_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")

def largest_press():
    # print("Largest button pressed")
    if ser and ser.is_open:
        ser.write(b'G')
    # Change button appearance
    disable_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    largest_step_button.config(highlightbackground="green", relief="sunken")
    middle_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    smallest_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")


def middle_press():
    # print("Middle button pressed")
    if ser and ser.is_open:
        ser.write(b'H')
    disable_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    largest_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    middle_step_button.config(highlightbackground="green", relief="sunken")
    smallest_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    

def smallest_press():
    # print("Smallest button pressed")
    if ser and ser.is_open:
        ser.write(b'J')
    disable_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    largest_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    middle_step_button.config(highlightbackground=root.cget('bg'), relief="sunken")
    smallest_step_button.config(highlightbackground="green", relief="sunken")

def on_key_press(event):
    if ser and ser.is_open:
        if event.keysym == 'Right':
            # print("Right arrow pressed")
            ser.write(b'R')
        elif event.keysym == 'Left':
            # print("Left arrow pressed")
            ser.write(b'L')
        elif event.keysym == 'Up':
            # print("Up arrow pressed")
            ser.write(b'U') 
        elif event.keysym == 'Down':
            # print("Down arrow pressed")
            ser.write(b'D') 

def on_key_release(event):
    if ser and ser.is_open:
        if event.keysym in ['Right', 'Left', 'Up', 'Down']:
            # print(f"{event.keysym} arrow released")
            ser.write(b'S')

selected_port = StringVar()
available_ports = refresh_ports()
default_port = "/dev/cu.usbmodem1101"
if default_port in available_ports:
    selected_port.set(default_port)
else:
    selected_port.set(available_ports[0] if available_ports else "None")

dropdown = OptionMenu(root, selected_port, *available_ports)
font_style = tkFont.Font(family="Arial", size=17)
dropdown.config(width=20, font=font_style)

connect_button = Button(root, text="CONNECT", command=on_connect)
connect_button.config(width=10)

# Create StringVar variables to hold X and Y text
x_var = StringVar(value="X: --")
y_var = StringVar(value="Y: --")

# Create labels and pack them
x_label = Label(root, textvariable=x_var, font=('Consolas', 24))
y_label = Label(root, textvariable=y_var, font=('Consolas', 24))

# Creates buttons
left_button = Button(root, text="X-")
right_button =Button(root, text="X+")
up_button = Button(root, text="Y+")
down_button =Button(root, text="Y-")
disable_step_button = Button(root, text="DISABLE STEP SIZE", command=disable_press)
largest_step_button = Button(root, text="1000", command=largest_press)
middle_step_button = Button(root, text="100", command=middle_press)
smallest_step_button =Button(root, text="10", command=smallest_press)

# Tracks GUI button presses and releases
left_button.bind('<ButtonPress>', left_press)
left_button.bind('<ButtonRelease>', left_release)
right_button.bind('<ButtonPress>', right_press)
right_button.bind('<ButtonRelease>', right_release)
up_button.bind('<ButtonPress>', up_press)
up_button.bind('<ButtonRelease>', up_release)
down_button.bind('<ButtonPress>', down_press)
down_button.bind('<ButtonRelease>', down_release)

# Enables keyboard arrow control
root.bind('<KeyPress-Right>', on_key_press)
root.bind('<KeyRelease-Right>', on_key_release)
root.bind('<KeyPress-Left>', on_key_press)
root.bind('<KeyRelease-Left>', on_key_release)
root.bind('<KeyPress-Up>', on_key_press)
root.bind('<KeyRelease-Up>', on_key_release)
root.bind('<KeyPress-Down>', on_key_press)
root.bind('<KeyRelease-Down>', on_key_release)

# TITLE
title_font = tkFont.Font(family="Arial", size=24, weight="bold")
title_label = Label(root, text="NANOPOSITIONER CONTROLS", font=title_font)
title_label.grid(row=0, column=0, columnspan=10, pady=20)

# CONNECTION
dropdown.grid(row=1, column=0, columnspan=7, pady=20)
connect_button.grid(row=1, column=7, columnspan=3, pady=20)
# refresh_button.grid(row=2, column=5, columnspan=4)

# POSITION LABEL
position_font = tkFont.Font(family="Arial", size=17)
position_label = Label(root, text="Position [µm]", font=position_font)
position_label.grid(row=4, column=0, columnspan=2, padx=30, pady=5, sticky="w")

# X & Y POSITION
x_label.grid(row=5, column=0, padx=10, pady=0, sticky="w")
y_label.grid(row=7, column=0, padx=10, pady=0, sticky="w")

# OPEN CONTROLS LABEL
open_controls_font = tkFont.Font(family="Arial", size=17)
open_controls = Label(root, text="Open Controls", font=open_controls_font)
open_controls.grid(row=4, column=5, columnspan=5, pady=5)

# OPEN CONTROLS
up_button.grid(row=5, column=7, padx=5, pady=5)
down_button.grid(row=7, column=7, padx=5, pady=5)
left_button.grid(row=6, column=5, padx=5, pady=5)
right_button.grid(row=6, column=8, padx=5, pady=5)

# STEP SIZE LABEL
step_size_font = tkFont.Font(family="Arial", size=17)
step_size_label = Label(root, text="Step Size [µm]", font=step_size_font)
step_size_label.grid(row=8, column=5, columnspan=5, pady=5)

# STEP SIZE CONTROLS
disable_step_button.grid(row=9, column=0, padx=5, pady=5)
largest_step_button.grid(row=9, column=5, padx=5)
middle_step_button.grid(row=9, column=7, padx=5, pady=5)
smallest_step_button.grid(row=9, column=8, padx=5, pady=5)

# Optional: make columns expand evenly on resize
root.grid_columnconfigure(0, weight=1)
root.grid_columnconfigure(1, weight=1)
root.grid_columnconfigure(2, weight=1)
root.grid_rowconfigure(0, weight=0)
root.grid_rowconfigure(1, weight=0)
root.grid_rowconfigure(2, weight=0)

def periodic_check():
    checkSerialPort()
    root.after(1, periodic_check)

periodic_check()
root.mainloop()
