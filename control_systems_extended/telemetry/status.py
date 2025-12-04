from pymavlink import mavutil
import time

def get_mode(vehicle):
    """
    Get the current flight mode of the vehicle.
    """
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        return mavutil.mode_mapping_acm[msg.custom_mode]
    return None

def is_armed(vehicle):
    """
    Check if the vehicle is armed.
    """
    msg = vehicle.recv_match(type='HEARTBEAT', blocking=True)
    if msg:
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    return False

def wait_for_heartbeat(vehicle):
    """
    Wait for the first heartbeat from the vehicle.
    """
    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()
    print("Heartbeat received.")

def get_global_position(vehicle):
    """
    Get the current global position (lat, lon, alt) of the vehicle.
    """
    msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        return (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0)
    return None
