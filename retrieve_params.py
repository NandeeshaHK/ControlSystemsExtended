from pymavlink import mavutil

def get_pixhawk_params(connection_string):
    try:
        # Create a MAVLink connection
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat()
        # Request parameter list
        master.param_fetch_all()

        # Wait for the parameter list to be received
        param_list = master.recv_match(type='PARAM_VALUE', blocking=True)

        # Print parameter names and values
        while param_list is not None:
            param_name = param_list.param_id
            param_value = param_list.param_value
            print(f"{param_name}: {param_value}")

            param_list = master.recv_match(type='PARAM_VALUE', blocking=True)

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    # Replace 'udp:127.0.0.1:14550' with the connection string of your Pixhawk
    connection_string = 'udp:127.0.0.1:14550'
    
    get_pixhawk_params(connection_string)
