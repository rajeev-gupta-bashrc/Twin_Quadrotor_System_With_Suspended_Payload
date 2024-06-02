from pymavlink import mavutil
import time
from geometry_msgs.msg import Vector3
import numpy as np

if __name__ == '__main__':
    altitude = 5
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udp:127.0.0.1:14560')
    # the_connection = mavutil.mavlink_connection('udp:localhost:14551')

    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("drone 1 :\n")
    print("Heartbeat from system (system %u component %u)" %
        (the_connection.target_system, the_connection.target_component))
    
    # arm
    print('Arming\n:')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # set mode guided
    print('Guided:\n')
    mode = 'GUIDED'
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(the_connection.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)

    # takeoff
    print('Takeoff:\n')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    
    while 1:
        pose = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        # Extract the position data from the message
        pos_x = pose.x  # meters
        pos_y = pose.y  # meters
        pos_z = pose.z  # meters (convert to upward positive convention)

        # Print the position data
        print(f"X: {pos_x} m, Y: {pos_y} m, Z: {pos_z} m")

        # msg = the_connection.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
        

        if (abs(pos_z - altitude) <= 0.01):
            break

    
    print('Done taking off\n')


