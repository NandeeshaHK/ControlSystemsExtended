from pymavlink import mavutil
import pandas as pd

# Connect to the Pixhawk (update the connection string)
connection_string = 'udp:127.0.0.1:14550'  # Replace with your connection string
vehicle = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat
vehicle.wait_heartbeat()

# Request mission items
vehicle.mav.mission_request_list_send(vehicle.target_system, vehicle.target_component)

# Wait for the mission count message
count_msg = vehicle.recv_match(type='MISSION_COUNT', blocking=True)
num_items = count_msg.count

# Request each mission item
commands = []
for seq in range(num_items):
    vehicle.mav.mission_request_send(vehicle.target_system, vehicle.target_component, seq)

    # Wait for the mission item message
    item_msg = vehicle.recv_match(type='MISSION_ITEM', blocking=True)
    commands.append(item_msg)

# Create a DataFrame to store mission commands
columns = ['Index', 'Latitude', 'Longitude', 'Altitude', 'Command', 'Param1', 'Param2', 'Param3', 'Param4', 'AutoContinue']
commands_df = pd.DataFrame(columns=columns)

# Populate DataFrame with mission command data
for cmd in commands:
    commands_df.loc[len(commands_df)] = [
        cmd.seq,
        cmd.x / 1e7,  # Coordinates are in degrees * 1e7
        cmd.y / 1e7,
        cmd.z,
        cmd.command,
        cmd.param1,
        cmd.param2,
        cmd.param3,
        cmd.param4,
        cmd.autocontinue
    ]

# Print the DataFrame
print("Mission Commands DataFrame:")
print(commands_df)

# Close connection
vehicle.close()
