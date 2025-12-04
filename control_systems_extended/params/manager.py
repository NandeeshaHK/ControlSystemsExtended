from pymavlink import mavutil

def get_all_params(connection):
    """
    Fetch all parameters from the vehicle.
    """
    connection.param_fetch_all()
    params = {}
    while True:
        msg = connection.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if not msg:
            break
        params[msg.param_id] = msg.param_value
    return params

def set_param(connection, param_name, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    """
    Set a parameter on the vehicle.
    """
    connection.mav.param_set_send(
        connection.target_system,
        connection.target_component,
        param_name.encode('utf-8'),
        param_value,
        param_type
    )
