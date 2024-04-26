from pymavlink import mavutil
import time 
# Connect to the Pixhawk
master = mavutil.mavlink_connection('tcp:127.0.0.1:5762')  # Adjust the connection string as per your setup
master.wait_heartbeat()
# Subscribe to mission item messages

# Subscribe to mission item reached messages
master.mav.mission_ack_send(master.target_system, master.target_component, mavutil.mavlink.MAV_MISSION_ACCEPTED)
while True:
    msg = master.recv_match(type=['MISSION_ITEM_REACHED'], blocking=True)
    if msg is not None:
        print("Reached Mission Item:", msg.seq)
        # Add more processing as needed
