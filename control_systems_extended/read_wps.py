from dronekit import connect, VehicleMode
import pandas as pd

# Connect to the Pixhawk (update the connection string)
connection_string = 'udp:127.0.0.1:14550'  # Replace with your connection string
vehicle = connect(connection_string, wait_ready=True)

# Download the mission commands from the Pixhawk
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()

# Create a DataFrame to store mission commands
columns = ['Index', 'Latitude', 'Longitude', 'Altitude', 'Command', 'Param1', 'Param2', 'Param3', 'Param4', 'AutoContinue']
commands_df = pd.DataFrame(columns=columns)

# Populate DataFrame with mission command data using loc
for cmd in cmds:
    commands_df.loc[len(commands_df)] = [
        cmd.seq,
        cmd.x,
        cmd.y,
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
