import paho.mqtt.client as mqtt
from datetime import datetime
import sys

# --- CONFIGURATION ---
BROKER_HOST = "192.168.0.74"  
BROKER_PORT = 1883
GROUP_ID = "13"   

# MiniMessenger typically formats topics as "GroupID/TargetBoard"
# Since the robot uses: messenger.sendToBoard("debug_console", message)
# The topic is usually GROUP_ID/debug_console
TARGET_TOPIC = f"{GROUP_ID}/debug_console"
WILDCARD_TOPIC = f"{GROUP_ID}/#" # Backup: listens to everything from your robot

LOG_FILE = "flight_log.txt"

def write_to_file(data):
    with open(LOG_FILE, "a") as f:
        f.write(data + "\n")

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"[SYSTEM] Connected to Broker at {BROKER_HOST}")
        print(f"[SYSTEM] Subscribing to telemetry stream...")
        client.subscribe(TARGET_TOPIC)
        
        # Uncomment the line below if you aren't seeing messages and want to spy on all traffic
        # client.subscribe(WILDCARD_TOPIC) 
    else:
        print(f"[FATAL] Connection failed with code {rc}")
        sys.exit(1)

def on_message(client, userdata, msg):
    # Ignore binary heartbeat pings, we only want text logs
    try:
        payload = msg.payload.decode('utf-8')
        
        # Format the timestamp
        now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        # Color formatting for terminal readability
        if "[ERROR]" in payload or "FATAL" in payload or "KILL SWITCH" in payload:
            terminal_out = f"\033[91m[{now}] {payload}\033[0m" # Red
        elif "[EVENT]" in payload or "[GPS]" in payload:
            terminal_out = f"\033[96m[{now}] {payload}\033[0m" # Cyan
        elif "[FSM]" in payload:
            terminal_out = f"\033[93m[{now}] {payload}\033[0m" # Yellow
        elif "[MISSION COMPLETE]" in payload:
            terminal_out = f"\033[92m\n[{now}] >>> {payload} <<<\n\033[0m" # Green
        else:
            terminal_out = f"[{now}] {payload}" # Default White

        print(terminal_out)
        
        # Save raw text to log file without the ANSI color codes
        write_to_file(f"[{now}] {payload}")
        
    except UnicodeDecodeError:
        pass # Silently drop raw binary packets

# --- MAIN EXECUTION ---
print("=========================================")
print(" KAYUBO FLIGHT TELEMETRY SYSTEM ONLINE")
print("=========================================")
write_to_file("\n\n--- NEW FLIGHT INITIALIZED ---")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

try:
    client.connect(BROKER_HOST, BROKER_PORT, 60)
    client.loop_forever() # Keeps the script running infinitely, waiting for packets
except KeyboardInterrupt:
    print("\n[SYSTEM] Telemetry Terminated by User.")
    sys.exit(0)
except Exception as e:
    print(f"\n[FATAL] Could not connect to broker. Error: {e}")