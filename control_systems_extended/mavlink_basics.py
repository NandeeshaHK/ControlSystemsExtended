from pymavlink import mavutil
import time

# the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
if __name__ == "__main__":
    the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=9600)

    # heartBeat check
    the_connection.wait_heartbeat()
    print('System heartbeat check ...')

    # mode to loiter
    the_connection.set_mode_loiter()
    time.sleep(3)

    # arm the drone
    the_connection.arducopter_arm()
    time.sleep(3)

    # disarm the drone
    the_connection.arducopter_disarm()
    time.sleep(3)

    # arm the drone
    the_connection.arducopter_arm()
    time.sleep(3)

    # mode to auto 
    the_connection.set_mode_auto()
