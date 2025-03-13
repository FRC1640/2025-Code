import subprocess
import time
from networktables import NetworkTables

username = "pi"
password = "raspberry"
NetworkTables.initialize(server='10.16.40.2') 
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
        for key, command in commands.items():
            result = subprocess.run(command, shell=True, capture_output=True, text=True)
            output = result.stdout.strip()
            if key == "temperature":
                diagnostics[key] = f"{int(output) / 1000:.2f}°C"
            else:
                diagnostics[key] = output
        opi.putNumber('CPU Temp', float(diagnostics['temperature'][:-2]))
        opi.putNumber('CPU Usage', float(diagnostics['cpu_usage']))
        opi.putNumber('Memory Usage', float(diagnostics['memory_usage']))
        opi.putNumber('Disk Usage', float(diagnostics['disk_usage'][:-1]))
    except Exception as e:
        print(f"Error: {e}")
        diagnostics = {key: "Error" for key in commands.keys()}
    
    return diagnostics

diagnostics = get_diagnostics()
while True:
    diagnostics = get_diagnostics()
    print(diagnostics)
    time.sleep(2)