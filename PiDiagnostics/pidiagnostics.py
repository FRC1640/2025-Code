import tkinter as tk
from tkinter import ttk
import paramiko
import threading
import logging
import subprocess
logging.basicConfig(filename='pidiagnostics.log', level=logging.INFO,
                    format='%(asctime)s - %(levelname)s - %(message)s')

username = "pi"
password = "raspberry"
hostname = "10.16.43.63"  

commands = {
    "temperature": "cat /sys/class/thermal/thermal_zone0/temp",
    "cpu_usage": "top -bn1 | grep 'Cpu(s)' | sed 's/.*, *\\([0-9.]*\\)%* id.*/\\1/' | awk '{print 100 - $1}'",
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
        client.close()
        logging.info("Diagnostics fetched successfully")
    except Exception as e:
        logging.error(f"Error fetching diagnostics: {e}")
        diagnostics = {key: "Error" for key in commands.keys()}
    return diagnostics

def update_diagnostics_labels():
    diagnostics = get_diagnostics()
    temperature_label.config(text=f"CPU Temperature: {diagnostics['temperature']}")
    cpu_usage_label.config(text=f"CPU Usage: {diagnostics['cpu_usage']}%")
    memory_usage_label.config(text=f"Memory Usage: {diagnostics['memory_usage']}%")
    disk_usage_label.config(text=f"Disk Usage: {diagnostics['disk_usage']}")
    root.after(1000, update_diagnostics_labels)  # Update every 1 second

def execute_ssh_command(command):
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect(hostname, username=username, password=password)
        stdin, stdout, stderr = client.exec_command(command)
        output = stdout.read().decode()
        client.close()
        logging.info(f"Executed SSH command: {command}")
        return output
    except Exception as e:
        logging.error(f"Error executing SSH command '{command}': {e}")
        return f"Error: {e}"

def on_enter_pressed(event):
    command = ssh_entry.get()
    ssh_entry.delete(0, tk.END)
    output = execute_ssh_command(command)
    ssh_terminal.insert(tk.END, f"$ {command}\n{output}\n")

root = tk.Tk()
root.title("Orange Pi Diagnostics Monitor")

temperature_label = ttk.Label(root, text="Fetching temperature...")
temperature_label.pack(pady=5)

cpu_usage_label = ttk.Label(root, text="Fetching CPU usage...")
cpu_usage_label.pack(pady=5)

memory_usage_label = ttk.Label(root, text="Fetching memory usage...")
memory_usage_label.pack(pady=5)

disk_usage_label = ttk.Label(root, text="Fetching disk usage...")
disk_usage_label.pack(pady=5)

ssh_terminal_frame = ttk.Frame(root)
ssh_terminal_frame.pack(pady=10, fill=tk.BOTH, expand=True)

ssh_terminal = tk.Text(ssh_terminal_frame, height=10, wrap=tk.WORD)
ssh_terminal.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

ssh_scrollbar = ttk.Scrollbar(ssh_terminal_frame, orient=tk.VERTICAL, command=ssh_terminal.yview)
ssh_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

ssh_terminal.config(yscrollcommand=ssh_scrollbar.set)

ssh_entry = ttk.Entry(root)
ssh_entry.pack(fill=tk.X, padx=10, pady=5)
ssh_entry.bind("<Return>", on_enter_pressed)

# Start the diagnostics update loop
update_diagnostics_labels()

# Run the GUI event loop
root.mainloop()