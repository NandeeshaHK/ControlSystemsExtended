from pymavlink import mavutil
import time
from .loader import load_waypoints_to_csv

class MissionManager:
    def __init__(self, connection):
        self.connection = connection
        self.wp_loader = mavutil.mavwp.MAVWPLoader()

    def upload_mission(self, mission_items):
        """
        Upload a mission to the vehicle.
        """
        self.wp_loader.clear()
        # ... logic to add items to loader ...
        
        self.connection.waypoint_clear_all_send()
        self.connection.waypoint_count_send(self.wp_loader.count())
        
        # ... logic to send items ...
        print("Mission uploaded.")

    def start_mission(self):
        """
        Set mode to AUTO to start mission.
        """
        self.connection.set_mode_auto()

    def clear_mission(self):
        """
        Clear the current mission.
        """
        self.connection.waypoint_clear_all_send()
