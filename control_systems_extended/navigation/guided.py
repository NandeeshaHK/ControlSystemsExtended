from dronekit import VehicleMode, LocationGlobalRelative
import time

def guided_takeoff(vehicle, altitude):
    """
    Arm and takeoff in GUIDED mode using DroneKit.
    """
    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off...")
    vehicle.simple_takeoff(altitude)

    while True:
        print(f" Altitude: {vehicle.location.global_relative_frame.alt}")
        if vehicle.location.global_relative_frame.alt >= altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def move_velocity(vehicle, velocity_x, velocity_y, velocity_z):
    """
    Move the vehicle with specified velocity components.
    """
    # Note: This requires sending MAVLink messages directly for velocity control in GUIDED mode
    # For simplicity in this SDK version, we'll use simple_goto with relative offsets if needed,
    # or implement the SET_POSITION_TARGET_LOCAL_NED message.
    pass # Placeholder for advanced velocity control
