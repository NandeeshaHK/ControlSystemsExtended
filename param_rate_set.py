from pymavlink import mavutil

# Connect to the MAVLink system (replace 'udp:127.0.0.1:14550' with your connection string)
mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
mavlink_connection.wait_heartbeat()
print('Got heart beat')
# Set custom rates for sr2_extra1 and sr2_position (replace 10 and 5 with your desired rates)
custom_rate_extra1 = 4
custom_rate_position = 4

# Send the parameter set command for sr2_extra1
mavlink_connection.mav.param_set_send(
    mavlink_connection.target_system,
    mavlink_connection.target_component,
    b'SR0_EXTRA1',  # Parameter name
    custom_rate_extra1,  # Custom rate value
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16  # Parameter type
)

# Send the parameter set command for sr2_position
mavlink_connection.mav.param_set_send(
    mavlink_connection.target_system,
    mavlink_connection.target_component,
    b'SR0_POSITION',  # Parameter name
    custom_rate_position,  # Custom rate value
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16  # Parameter type
)

mavlink_connection.mav.param_set_send(
    mavlink_connection.target_system,
    mavlink_connection.target_component,
    b'SR0_EXT_STAT',  # Parameter name
    custom_rate_position,  # Custom rate value
    mavutil.mavlink.MAV_PARAM_TYPE_UINT16  # Parameter type
)


exit()
# Set the waypoint speed in m/s (replace with your desired speed)
waypoint_speed = 10.0

# Send the MAV_CMD_DO_CHANGE_SPEED command to change the speed
mavlink_connection.mav.command_long_send(
    mavlink_connection.target_system,
    mavlink_connection.target_component,
    mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
    0,  # Confirmation
    0,  # Param1 (Unused for MAV_CMD_DO_CHANGE_SPEED)
    waypoint_speed,  # Param2: Speed in m/s
    -1,  # Param3 (Unused for MAV_CMD_DO_CHANGE_SPEED)
    0,  # Param4 (Unused for MAV_CMD_DO_CHANGE_SPEED)
    0,  # Param5 (Unused for MAV_CMD_DO_CHANGE_SPEED)
    0,  # Param6 (Unused for MAV_CMD_DO_CHANGE_SPEED)
    0   # Param7 (Unused for MAV_CMD_DO_CHANGE_SPEED)
)

# Wait for acknowledgment (you may need to adjust the timeout)
ack_msg = mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)

# Check if the command was accepted
if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print(f"Waypoint speed set to {waypoint_speed} m/s successfully.")
else:
    print("Failed to set waypoint speed.")