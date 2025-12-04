import os
from control_systems_extended.core import connect_mavlink
from control_systems_extended.mission import MissionManager, loader

def main():
    # Connect to the vehicle
    connection_string = 'udp:127.0.0.1:14550'
    vehicle = connect_mavlink(connection_string)
    
    manager = MissionManager(vehicle)

    # Example: Load waypoints from a file (assuming format matches loader expectations)
    # mission_items = loader.read_mission_file('path/to/mission.txt')
    
    # For demo, we'll just clear the mission
    print("Clearing current mission...")
    manager.clear_mission()
    
    # In a real scenario:
    # manager.upload_mission(mission_items)
    # manager.start_mission()

if __name__ == "__main__":
    main()
