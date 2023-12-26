from pymavlink import mavutil

def get_pixhawk_param(connection_string, param_name):
    try:
        # Create a MAVLink connection
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
        # Request the specific parameter
        master.param_fetch_one(param_name)

        # Wait for the parameter value to be received
        param_value_msg = master.recv_match(type='PARAM_VALUE', blocking=True)

        # Print the parameter name and value
        if param_value_msg is not None:
            param_value = param_value_msg.param_value
            print(f"{param_name}: {param_value}")
        else:
            print(f"Parameter '{param_name}' not found.")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Replace 'udp:127.0.0.1:14550' with the connection string of your Pixhawk
    connection_string = 'udp:127.0.0.1:14550'
    
    # Replace 'SPECIFIC_PARAM_NAME' with the name of the parameter you want to retrieve
    param_name_to_get = 'SR0_POSITION'
    param_name_to_get2 = 'SR0_EXTRA1'
    param_name_to_get3 = 'SR0_EXT_STAT'

    get_pixhawk_param(connection_string, param_name_to_get)
    get_pixhawk_param(connection_string, param_name_to_get2)
    get_pixhawk_param(connection_string, param_name_to_get3)
