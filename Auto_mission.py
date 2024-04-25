#!/usr/bin/env python3
import os
import time
import asyncio
import numpy as np
import pandas as pd
from pymavlink import mavwp
from pymavlink import mavutil
from math import cos,sin,atan2,radians,sqrt

# set the directory where the .waypoints files are located
#waypoints_dir = '/home/raspi/Waypoint-auto-mission/waypoint/' #for raspi
waypoints_dir = '/home/nerdnhk/upload_mission/wpf_folder/'

# set the directory where the output .csv files will be saved
#csv_dir = '/home/raspi/Waypoint-auto-mission/csv/' #for raspi
csv_dir = '/home/nerdnhk/upload_mission/csv_files/'


# fill MAVLINK ID one use '/dev/ttyusbx' (x is a int), 'UDP' or 'TCP' #udpin://:14550 
MAVLINK_ID1 = "udpin:localhost:14550"
baud1 = 57600

# Fill the required range, default 100m
wp_range = 100 

# Start a connection listening on a UDP port
print("Trying to establish a MavLink connection with the Drone...")
the_connection = mavutil.mavlink_connection(MAVLINK_ID1)

# Wait for the first heartbeat 
print("Waiting For HeartBeat")
the_connection.wait_heartbeat()

# This sets the system and component ID of remote system for the link
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

print("Loading Waypoint Loader")
wp = mavwp.MAVWPLoader() 

no_tries = 0
wp_list = []
wp_file_list = []
file_dis_list = []
csv_file_list = []
curr_lat = np.float32(0)
curr_lon = np.float32(0)
alt = np.float32(0)
wpf_final = ""

def init_conn(the_connection):
    #print("System is armed.")
    msg = the_connection.recv_match(type="GLOBAL_POSITION_INT",blocking = True)
    curr_lat = (msg.lat)/1e7
    curr_lon = (msg.lon)/1e7

    #return current gps
    return (curr_lat,curr_lon)

#a command function to set home location
def cmd_set_home(home_location, altitude):
    print('--- ', the_connection.target_system, ',', the_connection.target_component)
    the_connection.mav.command_long_send(
        the_connection.target_system, the_connection.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        1,  # set position
        0,  # param1
        0,  # param2
        0,  # param3
        0,  # param4
        home_location[0],  # lat
        home_location[1],  # lon
        altitude)

def distance_lat_lon(lat1, lon1, lat2, lon2):
    '''distance between two points'''
    dLat = radians(lat2) - radians(lat1)
    dLon = radians(lon2) - radians(lon1)
    a = sin(0.5*dLat)**2 + sin(0.5*dLon)**2 * cos(lat1) * cos(lat2)
    c = 2.0 * atan2(sqrt(abs(a)), sqrt(abs(1.0-a)))
    ground_dist = 6371 * 1000 * c
    return ground_dist

def wpToCSV(waypoints_dir):
    print("Converting waypoints file to CSV")
    for filename in os.listdir(waypoints_dir):
        #print("Converting "+str(filename)+" to CSV")
        if filename.endswith('.waypoints'):
            # read the file into a pandas dataframe
            df = pd.read_csv(os.path.join(waypoints_dir, filename), delimiter='\t', header=None, names=['seq', 'current','frame', 'command','param_1', 'param_2', 'param_3', 'param_4', 'latitude', 'longitude', 'altitude','mission_type'])

            # construct the output filename
            output_filename = os.path.splitext(filename)[0] + '.csv'

            # write the selected columns to a new .csv file
            df.to_csv(os.path.join(csv_dir, output_filename), index=False)

def ext_gps(csv_file_path,gps):
    # Extract 3 columns from CSV file
    df = pd.read_csv(csv_file_path, usecols=['command', 'latitude','longitude'])
    f_lat = np.float32(0)
    f_lon = np.float32(0)
    
    for i in range(1,3):
        row = df.loc[i, :]
        if row['command'] !=22 and row.loc['latitude'] != 0.0 and row.loc['latitude'] !=0.0:
            f_lat = row.loc['latitude']
            f_lon = row.loc['longitude']
            break
    dis = distance_lat_lon(f_lat,f_lon,np.float32(gps[0]),np.float32(gps[1]))
    print("Found distance is "+str(dis/1000)+"km")
    file_dis_list.append(dis)


# to find the shortest distance
def final_wpf(GPS):
    file_name_list = os.listdir(csv_dir)
    print(file_name_list)
    for file_name in file_name_list:
        csv_file_path = os.path.join(csv_dir,file_name)
        if os.path.isfile(csv_file_path):
            print(file_name+" CSV file found")
            (ext_gps(csv_file_path,GPS))
        else:
            print("CSV file not found")
    # to find minium distance WP file
    min_dis_wpf = min(file_dis_list)
    print(min_dis_wpf)
    if min_dis_wpf <= wp_range:
        # less than 100m
        print("Drone is in range of 100m for the Shortest waypoint file")
    else:
        print("Drone is NOT in range of 100m for the Shortest waypoint file")
        # Empty the list for future USE
        file_dis_list.clear()
        return '\0'
    
    # convert to Pandas Series for easy conversion
    pd_series = pd.Series(file_dis_list)
    min_dis_index = pd_series.index[pd_series == min_dis_wpf][0]
    print("The shortest distance wp file:"+str(min_dis_index))
    
    # now file _name_list consists only waypoints
    file_name_list = os.listdir(csv_dir)
    wpf_final = file_name_list[min_dis_index]
    print("Final WP file seleted: "+wpf_final+"\n")

    # Empty the list for future USE
    file_dis_list.clear()
    return wpf_final

def uploadmission(aFileName,gps):
    #home_location = [curr_lat,curr_lon]
    home_altitude = None
    total_lines = 0
    with open(aFileName) as f2:
        for i, line in enumerate(f2):
            total_lines = i-2
        print("Total number of lines: "+str(total_lines))

    with open(aFileName) as f:
        next(f)
        for i, line in enumerate(f): #i iterates as normal int and line iterates every line in the file
            if i == 0:
                print("Started Waypoint Parsing")
            else:
                #split all the words in the line
                linearray = line.strip().split(',')
                #assigning every param to a variable
                ln_seq = int(linearray[0])
                ln_current = int(round(float(linearray[1])))
                ln_frame = int(round(float(linearray[2])))
                ln_command = int(round(float(linearray[3])))
                ln_param1 = float(linearray[4])
                ln_param2 = float(linearray[5])
                ln_param3 = float(linearray[6])
                ln_param4 = float(linearray[7])
                ln_x = int(float(linearray[8])*1e7)
                ln_y = int(float(linearray[9])*1e7)
                ln_z = float(linearray[10])
                ln_autocontinue = int(round(float(linearray[11])))
                if i == 2 and ln_command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    home_altitude = int(ln_z)
                    ln_command = mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                    print("Changed first WayPoint to TakeOff ")
                if i == 1:
                    print("Changed Home Location to current Drone Location")
                    ln_x = int(round(float(gps[0]))*1e7)
                    ln_y = int(round(float(gps[1]))*1e7)
                '''
                if ln_seq == 11 and ln_command == mavutil.mavlink.MAV_CMD_NAV_WAYPOINT:
                    home_altitude = int(ln_z)
                    ln_command = mavutil.mavlink.MAV_CMD_NAV_LAND
                    print("Changed last WayPoint to LAND ")
                '''        
                print(f"Waypoint: ({ln_command}, {ln_x/1e7}, {ln_y/1e7}, {ln_z})")
                p = mavutil.mavlink.MAVLink_mission_item_int_message(the_connection.target_system, the_connection.target_component, ln_seq,
                                                                 ln_frame,
                                                                 ln_command,
                                                                 ln_current, ln_autocontinue, ln_param1, ln_param2,
                                                                 ln_param3, ln_param4, ln_x, ln_y, ln_z, 0)
                wp.add(p)
    # send waypoint to airframe
    the_connection.waypoint_clear_all_send()
    the_connection.waypoint_count_send(wp.count())
    print()
    
    for i in range(wp.count()):
        msg = the_connection.recv_match(type=['MISSION_REQUEST'], blocking=True)
        #print(msg)
        the_connection.mav.send(wp.wp(msg.seq))
        print('Sending waypoint {0}'.format(msg.seq))
    msg = the_connection.recv_match(type=['MISSION_ACK'], blocking=True)
    #to print the recieved cmd acknowledge and set home location
    print(msg)
    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
    	print("Mission Uploaded successfully")

def set_RTL(drone):
        # Set the drone mode to RTL
    drone.mav.command_long_send(
        drone.target_system,                  # target_system
        drone.target_component,                  # target_component
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
        0,                  # confirmation
        0, 0, 0, 0, 0, 0,   # params 1-6
        1)                  # param 7: set to 1 to enable RTL mode
	

def arm(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     400, 0, 1, 0, 0, 0, 0, 0, 0)


def takeoff(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == 0:
        print("Command 'TAKEOFF' executed successfully")
    else:
        print("Command failed with error code:", msg.result)
        print("Trying once again to takeoff")
        no_tries += 1
        if no_tries == 3:
            no_tries = 0
            return 0
        takeoff(the_connection)

def changetoloiter(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     176, 0, 1, 5, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == 0:
        print("Command 'LOITER' executed successfully")
    else:
        print("Command failed with error code:", msg.result)

def changetoauto(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     176, 0, 1, 3, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == 0:
        print("Command 'AUTO' executed successfully")
    else:
        print("Command failed with error code:", msg.result)

def changetoguided(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     176, 0, 1, 4, 0, 0, 0, 0, 0)

    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

def last_sequence(file):
        last_seq = 0
        with open(file) as f2:
            for i, line in enumerate(f2):
                last_seq = i-2
            print("Total number of Waypoints: "+str(last_seq))
        return last_seq

def wait_positive_arm(the_connection):
    prev_GPS = (0,0)
    valid_wp = 0
    prev_wp_file_list = os.listdir(waypoints_dir)
    count = 0  
    while valid_wp == 0:
        wp_file_list = os.listdir(waypoints_dir)
        if prev_wp_file_list != wp_file_list:
            wpToCSV(waypoints_dir)
            prev_wp_file_list = wp_file_list
        # get_gps data
        GPS = init_conn(the_connection)
        count += 1
        if count == 1000:
            print("GPS fetched for 1000 times, still no fix, going to terminate program")
            return '\0'
        if prev_GPS != GPS or valid_wp == 0:
            print("-- Coordinates of drone: lat:"+str(GPS[0])+" and lon:"+str(GPS[1]))
            prev_GPS = GPS
            wpf_final = final_wpf(GPS)
            if wpf_final == '\0':
                valid_wp = 0
            else:
                print("Got Valid Waypoint File:"+str(wpf_final))
                valid_wp = 1

        time.sleep(0.5)
    # check if the command succeeded
    if valid_wp == 1:
        #print('-- Valid waypoint found')
        return wpf_final
    else:
        print('-- Checking VALID waypoint')
        wait_positive_arm(the_connection)

def beep():

    # Send a MAV_CMD_DO_SET_RELAY command to make the buzzer beep
    relay_id = 0  # Relay ID for the Pixhawk's onboard buzzer
    duration = 1000  # Duration of each beep in milliseconds
    interval = 1000  # Interval between beeps in milliseconds
    repeat = 1  # Number of times to repeat the beep sequence
    command = the_connection.mav.command_long_encode(
        1,  # Target system
        1,  # Target component
        mavutil.mavlink.MAV_CMD_DO_SET_RELAY,  # Command ID
        0,  # Confirmation
        relay_id,  # Relay ID
        repeat,  # Number of times to repeat the beep sequence
        duration,  # Duration of each beep in milliseconds
        interval,  # Interval between beeps in milliseconds
        0, 0, 0)  # Parameters 4-6 (not used)
    the_connection.mav.send(command)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_RELAY:
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print('Buzzer beep command accepted')
        else:
            print('Buzzer beep command rejected')
    # Wait for the beeps to finish
    time.sleep((duration + interval) / 1000.0 * repeat)

def check_global_pos(the_connecition):
    # Wait for the drone to have a global position estimate
    print("Waiting for drone to have a global position estimate...")
    while True:
        print("receiving HEART BEAT")
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if 'GLOBAL_POSITION_INT' in the_connecition.messages and 'HOME_POSITION' in the_connection.messages:
            print("-- Global position estimate OK")
            break

async def run():
    beep()
    #Convert all the waypoints to csv files
    wpToCSV(waypoints_dir)
    print("-- Drone set to GUIDE Mode")
    changetoguided(the_connection)
    print("-- Waiting for 5 seconds")
    for i in range(5):
        time.sleep(1)
        print(5-i)
    print("-- Getting RealTime GPS Coordinates ")
    wpf = wait_positive_arm(the_connection)
    
    # Checking file is in 100m range
    if wpf_final == '\0':
        print("At armed position, there is NO VALID waypoint File in 100m Range: exit 0")
        exit()
    # get_gps data
    GPS = init_conn(the_connection)

    # Uploading Mission
    print(str(wpf)+"before upload")
    file_path = os.path.join(csv_dir, wpf)
    uploadmission(file_path,GPS)

    #change to guide MODE command
    print("Changing to LOITER Mode")
    changetoloiter(the_connection)
    '''
    #change to loiter MODE command
    print("-- Drone set to Loiter MODE")
    changetoloiter(the_connection)
    '''
    # Arming command
    print("-- Arming")
    arm(the_connection)
    await asyncio.sleep(2)

    #check GLOBAL POSITION ESTIMATE
    check_global_pos(the_connection)

    #command to take off
    print("-- Taking off")
    takeoff(the_connection)
    await asyncio.sleep(1)

    #command to set to AUTO
    print("-- Drone set to Auto MODE")
    changetoauto(the_connection)

    last_seq = last_sequence(file_path)
    prev_item = -1
    # Main loop to monitor mission progress
    while True:
        # Wait for a new message from the autopilot
        msg = the_connection.recv_match()
        if not msg:
            continue

        # Check if the message is the current mission item
        if msg.get_type() == 'MISSION_CURRENT':
            if prev_item != msg.seq:
                prev_item = msg.seq
                print('Current mission item:', msg.seq)
        
            # Termiante if it reaches end of the waypoint 
            if last_seq == msg.seq:
                set_RTL(the_connection)
                print("-- Set to Return To Launch")
                # delete the directory except required CSV
                file_name_list = os.listdir(csv_dir)
                for file_name in file_name_list:
                    csv_file_path = os.path.join(csv_dir,file_name)
                    os.remove(csv_file_path)
                break
    # Ending connection
    the_connection.close()



if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())

