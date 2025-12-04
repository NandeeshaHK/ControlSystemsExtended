from pymavlink import mavutil
import time

# Connect to the drone (replace 'udp:127.0.0.1:14550' with your connection string)
connection_string = 'udp:127.0.0.1:14550'
vehicle = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat
while not vehicle.wait_heartbeat():
    print("Waiting for heartbeat...")
    time.sleep(1)

# Function to get the current mode of the drone
def get_current_mode():
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)

    return mavutil.mode_mapping_acm[msg.custom_mode]


# Get and print the current mode
while True:
    mode = get_current_mode()

    if mode is not None:
        print(f"Current Mode: {mode}")

# Close the connection to the drone
# vehicle.close()
