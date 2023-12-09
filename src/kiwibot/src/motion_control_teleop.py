#!/usr/bin/env python
import rospy
import numpy as np
import socket
from time import sleep
from kiwibot.msg import ctrl_vec


throttle = 0.5

def teleop():
    print('Starting server...')
    # host = 'spectre.local'
    host = '192.168.12.1'
    port = 10000
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print('Listening for connections {host}:{port}')
    client_socket, client_address = server_socket.accept()

    pub = rospy.Publisher('cmd_vel', ctrl_vec, queue_size=10)
    rate = rospy.Rate(30)
    throttle = 1.0
    while not rospy.is_shutdown():
        translation = np.array([0.0, 0.0])
        msg = ctrl_vec()
        
        # Wait for data from client (ESP32 motion controller)
        data = client_socket.recv(2048).decode('utf-8')
        if len(data) == 0:
            continue

        elif 'Q' in data:
            print('Quitting...')
            client_socket.shutdown(1)
            client_socket.close()
            msg.speed = 0
            msg.dir = 0
            msg.rot_speed = 0
            pub.publish(msg)
            msg = ctrl_vec()
            msg.cmd = 'quit'
            pub.publish(msg)
            exit()
            break
        elif data[0] == 'M':
            # try:
            print(data)
            data.replace('M', ' ')
            data.replace('\r\n', ' ')
            vals = data.split(' ')
            print(vals)
            msg.speed = throttle

            dir = parse_dir(vals[1])
            if dir < 0:
                msg.speed = 0
                msg.dir = 0
            else:
                msg.dir = dir
            
            # msg.rot_speed = parse_rot(vals[2])
            msg.rot_speed = 0
            print(msg.rot_speed)
            # except:
            #     print('Invalid data')
        pub.publish(msg)
        rate.sleep()
    server_socket.shutdown(1)
    server_socket.close()

def parse_dir(dir_str):
    # dir_str = str(dir_str)
    if dir_str == 'E':
        return 180
    elif dir_str == 'NE':
        return 135
    elif dir_str == 'N':
        return 90
    elif dir_str == 'NW':
        return 45
    elif dir_str == 'W':
        return 0
    elif dir_str == 'SW':
        return 315
    elif dir_str == 'S':
        return 270
    elif dir_str == 'SE':
        return 225
    else:
        return -1
    
def parse_rot(dir_str):
    dir_str = str(dir_str)
    print(dir_str)
    if 'CW' in dir_str:
        return -3
    elif 'CCW' in dir_str:
        return 3
    else:
        return 0


if __name__ == '__main__':
    rospy.init_node('motion_control_teleop', anonymous=True)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass