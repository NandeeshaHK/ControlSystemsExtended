from pymavlink import mavutil

def is_drone_armed(connection_string):
    try:
        # Establish a connection to the autopilot
        master = mavutil.mavlink_connection(connection_string)

        # Wait for the HEARTBEAT message
        msg = None
        while msg is None or msg.get_type() != 'HEARTBEAT':
            msg = master.recv_msg()

        # Check if the drone is armed
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    except Exception as e:
        print(f"Error: {e}")
        return False


# Example usage
connection_string = 'udp:127.0.0.1:14550'  # Replace with the appropriate connection string
armed_status = is_drone_armed(connection_string)

if armed_status:
    print("The drone is armed.")
else:
    print("The drone is not armed.")
