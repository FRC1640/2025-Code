import os
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
import paramiko
import subprocess
import threading
import ttkbootstrap as tb
from networktables import NetworkTables
from PIL import ImageTk, Image

username = "pi"
password = "raspberry"
hostname = "10.16.40.63"
server_ip = "10.16.40.2"

NetworkTables.initialize(server=server_ip) 
opi = NetworkTables.getTable('Orange PI Diagnostics')

commands = {
    "temperature": "cat /sys/class/thermal/thermal_zone0/temp",
    "cpu_usage": "top -bn1 | grep 'Cpu(s)' | awk '{print 100 - $8}'",
    "memory_usage": "free -m | awk 'NR==2{printf \"%.2f\", $3*100/$2 }'",
    "disk_usage": "df -h | awk '$NF==\"/\"{printf \"%s\", $5}'"
}

def get_diagnostics():
    diagnostics = {}
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(hostname, username=username, password=password)

        for key, command in commands.items():
            stdin, stdout, stderr = client.exec_command(command)
            output = stdout.read().decode().strip()
            if key == "temperature":
                diagnostics[key] = f"{int(output) / 1000:.2f}°C"
            else:
                diagnostics[key] = output

        opi.putNumber('CPU Temp', float(diagnostics['temperature'][:-2]))
        opi.putNumber('CPU Usage', float(diagnostics['cpu_usage']))
        opi.putNumber('Memory Usage', float(diagnostics['memory_usage']))
        opi.putNumber('Disk Usage', float(diagnostics['disk_usage'][:-1]))
        client.close()
    except Exception as e:
        print(f"Goobersnort Error: {e}")
        diagnostics = {key: "Gooberstorn Error" for key in commands.keys()}
    
    return diagnostics

def update():
    def threaded_update():
        diagnostics = get_diagnostics()
        if diagnostics["temperature"] != "Goobersnort Error" and float(diagnostics["temperature"][:-2]) >= 85:
            temperature_label.config(text=f"CPU Temperature: Unplug immediately its a bit toasty")
        else:
            temperature_label.config(text=f"CPU Temperature: {diagnostics['temperature']}")

        cpu_usage_label.config(text=f"CPU Usage: {diagnostics['cpu_usage']}%")
        memory_usage_label.config(text=f"Memory Usage: {diagnostics['memory_usage']}%")
        disk_usage_label.config(text=f"Disk Usage: {diagnostics['disk_usage']}")

        root.after(1000, update)
    
    threading.Thread(target=threaded_update, daemon=True).start()

def execute_ssh_command(command):
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(hostname, username=username, password=password)

        stdin, stdout, stderr = client.exec_command(command)
        output = stdout.read().decode()
        client.close()

        return output
    except Exception as e:
        return f"Error: {e}"

def open_ssh_terminal():
    ssh_command = f"ssh {username}@{hostname}"
    subprocess.Popen(['start', 'cmd', '/k', ssh_command], shell=True)

def update_settings():
    global hostname, server_ip
    hostname = hostname_entry.get()
    server_ip = server_entry.get()
    NetworkTables.initialize(server=server_ip)
    status_label.config(text=f"Server: {server_ip}, Host: {hostname}")
    get_diagnostics()

root = tb.Window(themename="darkly")
root.title("Orange Pi Diagnostics Monitor")
root.geometry("480x600")

frame_main = ttk.Frame(root, padding=15)
frame_main.pack(fill="both", expand=True)

title_label = tb.Label(frame_main, text="Orange Pi Diagnostics", font=("Arial", 18, "bold"))
title_label.pack(pady=10)

temperature_label = tb.Label(frame_main, text="Fetching temperature...", font=("Arial", 12))
temperature_label.pack(pady=5)

cpu_usage_label = tb.Label(frame_main, text="Fetching CPU usage...", font=("Arial", 12))
cpu_usage_label.pack(pady=5)

memory_usage_label = tb.Label(frame_main, text="Fetching memory usage...", font=("Arial", 12))
memory_usage_label.pack(pady=5)

disk_usage_label = tb.Label(frame_main, text="Fetching disk usage...", font=("Arial", 12))
disk_usage_label.pack(pady=5)

ssh_button = tb.Button(frame_main, text="Open SSH Terminal", command=open_ssh_terminal, bootstyle="primary")
ssh_button.pack(pady=10)

hostname_label = tb.Label(frame_main, text="Host IP:")
hostname_label.pack()
hostname_entry = tb.Entry(frame_main)
hostname_entry.insert(0, hostname)
hostname_entry.pack()

server_label = tb.Label(frame_main, text="Server IP:")
server_label.pack()
server_entry = tb.Entry(frame_main)
server_entry.insert(0, server_ip)
server_entry.pack()

update_button = tb.Button(frame_main, text="Update Settings", command=update_settings, bootstyle="success")
update_button.pack(pady=5)

status_label = tb.Label(frame_main, text="")
status_label.pack()

script_dir = os.path.dirname(os.path.abspath(__file__))
icon_path = os.path.join(script_dir, "resources", "icon.png")
if os.path.exists(icon_path):
    img = ImageTk.PhotoImage(Image.open(icon_path))
    panel = tk.Label(root, image=img)
    panel.pack(side="bottom", fill="both", expand="yes")
    icon = tk.PhotoImage(file=icon_path)
    root.iconphoto(False, icon)
else:
    print("Warning: Icon file not found.")

update()

root.mainloop()
