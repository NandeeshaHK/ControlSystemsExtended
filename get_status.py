from pymavlink import mavutil
import time
import threading

# Connect to the drone (replace 'udp:127.0.0.1:14550' with your connection string)
connection_string = 'udp:127.0.0.1:14550'
master = mavutil.mavlink_connection(connection_string)

# Wait for the vehicle to be ready
while not master.wait_heartbeat().system_status == mavutil.mavlink.MAV_STATE_STANDBY:
    print("Waiting for vehicle to initialize...")
    time.sleep(1)

# Listener function for STATUSTEXT messages
def statustext_callback(msg):
    print(f"Status Text: {msg.text}")

# Function to process incoming messages
def process_messages():
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True)
        if msg:
            statustext_callback(msg)

# Start a thread to process incoming messages
message_thread = threading.Thread(target=process_messages)
message_thread.start()

# Function to keep the program running
def keep_program_running():
    while True:
        time.sleep(1)

# Call the function to keep the program running
keep_program_running()
