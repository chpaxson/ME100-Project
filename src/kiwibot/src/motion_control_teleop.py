#!/usr/bin/env python
import rospy
import numpy as np
import socket
from time import sleep
from kiwibot.msg import ctrl_vec

def teleop():
    print("Starting server...")
    host = 'spectre.local'
    port = 10000
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print("Listening for connections {host}:{port}")
    client_socket, client_address = server_socket.accept()

    pub = rospy.Publisher('cmd_vel', ctrl_vec, queue_size=10)
    rate = rospy.Rate(30)
    throttle = 1.0
    while not rospy.is_shutdown():
        translation = np.array([0.0, 0.0])
        msg = ctrl_vec()
        
        # Wait for data from client (ESP32 motion controller)
        data = client_socket.recv(1024).decode('utf-8')
        if len(data) == 0:
            continue
        elif data[0] == "Q":
            print("Quitting...")
            client_socket.shutdown(1)
            client_socket.close()
            msg.cmd = "stop"
            pub.publish(msg)
            exit()
            break
        elif data[0] == "M":
            try:
                # print(data)
                vals = data[1:].split(" ")
                print(vals)
                translation[0] = -float(vals[1])
                translation[1] = float(vals[2])
                # Normalize translation vector
                if np.linalg.norm(translation) > 0.0:
                    translation = translation / np.linalg.norm(translation)
                yaw = float(vals[3])
                # rotation = -yaw
                rotation = 0
                msg.speed = np.linalg.norm(translation)
                msg.dir = np.arctan2(translation[1], translation[0])
                msg.rot_speed = rotation
            except:
                print("Invalid data")
        pub.publish(msg)
    server_socket.shutdown(1)
    server_socket.close()


if __name__ == '__main__':
    rospy.init_node('motion_control_teleop', anonymous=True)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass