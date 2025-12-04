from pymavlink import mavutil
from dronekit import connect

def connect_mavlink(connection_string, baud=57600):
    """
    Establish a MAVLink connection using pymavlink.
    """
    print(f"Connecting to {connection_string} via MAVLink...")
    connection = mavutil.mavlink_connection(connection_string, baud=baud)
    connection.wait_heartbeat()
    print(f"Heartbeat from system (system {connection.target_system} component {connection.target_component})")
    return connection

def connect_dronekit(connection_string, wait_ready=True):
    """
    Establish a connection using dronekit.
    """
    print(f"Connecting to {connection_string} via DroneKit...")
    vehicle = connect(connection_string, wait_ready=wait_ready)
    print("DroneKit connection established.")
    return vehicle
