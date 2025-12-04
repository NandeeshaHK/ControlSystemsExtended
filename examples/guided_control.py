from control_systems_extended.core import connect_dronekit
from control_systems_extended.navigation import guided

def main():
    # Connect using DroneKit for guided control
    connection_string = '127.0.0.1:14550'
    print(f"Connecting to {connection_string}...")
    vehicle = connect_dronekit(connection_string)

    # Takeoff to 10 meters
    target_altitude = 10
    print(f"Taking off to {target_altitude}m...")
    guided.guided_takeoff(vehicle, target_altitude)

    print("Hovering for 5 seconds...")
    import time
    time.sleep(5)

    print("Landing...")
    from dronekit import VehicleMode
    vehicle.mode = VehicleMode("LAND")
    
    vehicle.close()

if __name__ == "__main__":
    main()
