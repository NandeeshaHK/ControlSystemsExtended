from pymavlink import mavutil
import time

def set_mode(vehicle, mode):
    """
    Set the flight mode of the vehicle.
    """
    # Check if mode is available in the map
    mode_id = vehicle.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return False

    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    return True

def arm(vehicle):
    """
    Arm the vehicle.
    """
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

def disarm(vehicle):
    """
    Disarm the vehicle.
    """
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

def takeoff(vehicle, altitude):
    """
    Command the vehicle to takeoff to a specified altitude.
    """
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude
    )

def land(vehicle):
    """
    Command the vehicle to land.
    """
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
