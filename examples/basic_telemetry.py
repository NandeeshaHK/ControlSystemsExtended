import time
from control_systems_extended.core import connect_mavlink
from control_systems_extended.telemetry import status

def main():
    # Connect to the vehicle
    connection_string = 'udp:127.0.0.1:14550'
    print(f"Connecting to {connection_string}...")
    vehicle = connect_mavlink(connection_string)

    # Wait for heartbeat
    status.wait_for_heartbeat(vehicle)

    # Monitor status for a few seconds
    for _ in range(5):
        mode = status.get_mode(vehicle)
        armed = status.is_armed(vehicle)
        pos = status.get_global_position(vehicle)
        
        print(f"Mode: {mode}, Armed: {armed}, Position: {pos}")
        time.sleep(1)

if __name__ == "__main__":
    main()
