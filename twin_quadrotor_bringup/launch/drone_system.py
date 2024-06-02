from pymavlink import mavutil
import time
from geometry_msgs.msg import Vector3
import numpy as np

if __name__ == '__main__':
    connection1 = mavutil.mavlink_connection('udp:127.0.0.1:14560')
    connection2 = mavutil.mavlink_connection('udp:127.0.0.1:14570')

    connection1.wait_heartbeat()
    connection2.wait_heartbeat()
    print("drone 1 :\n")
    print("Heartbeat from system (system %u component %u)" %
        (connection1.target_system, connection1.target_component))
    print("drone 2 :\n")
    print("Heartbeat from system (system %u component %u)" %
        (connection2.target_system, connection2.target_component))

    print('Arming1\n:')
    connection1.mav.command_long_send(connection1.target_system, connection1.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = connection1.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print('Arming2\n:')
    connection2.mav.command_long_send(connection2.target_system, connection2.target_component,
                                        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = connection2.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    print('Guided1:\n')
    mode1 = 'GUIDED'
    mode_id1 = connection1.mode_mapping()[mode1]
    connection1.mav.set_mode_send(connection1.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id1)
    print('Guided2:\n')
    mode2 = 'GUIDED'
    mode_id2 = connection2.mode_mapping()[mode2]
    connection2.mav.set_mode_send(connection2.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id2)


    takeoff_height = 3
    print('Takeoff1:\n')
    connection1.mav.command_long_send(connection1.target_system, connection1.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_height)
    msg = connection1.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    print('Takeoff2:\n')
    connection2.mav.command_long_send(connection2.target_system, connection2.target_component,
                                        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, takeoff_height)
    msg = connection2.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
    
    while 1:
        pose1 = connection1.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        pose2 = connection2.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        pos_x1 = pose1.x  
        pos_x2 = pose2.x  
        pos_y1 = pose1.y
        pos_y2 = pose2.y
        pos_z1 = pose1.z 
        pos_z2 = pose2.z 
        
        print(pos_z1, pos_z2)