#!/usr/bin/env python
import rospy
import socket
import numpy as np
from kiwibot.msg import ctrl_vec

host = 'kiwibot.local'
port = 12345

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((host, port))

mot_num = {"A": 0, "B": 1, "C": 2}

wheel_diam = 4 # in
wheel_diam = wheel_diam * 0.0254 # m
robot_radius = 0.1435 # m
max_wheel_rpm = 251 # rpm
A_ang = np.deg2rad(30 - 90)
B_ang = np.deg2rad(150 - 90)
C_ang = np.deg2rad(270 - 90)
unit_A = np.array([np.cos(A_ang), np.sin(A_ang)])
unit_B = np.array([np.cos(B_ang), np.sin(B_ang)])
unit_C = np.array([np.cos(C_ang), np.sin(C_ang)])

# Send control inputs to motors of robot
# A ------- B
#  \       /
#   \     /
#    \   /
#     \ /
#      C

def callback(message):
    print("Received message")
    if message.cmd == "quit":
        client_socket.send("QUIT".encode('utf-8'))
        return
    if message.cmd == "stop":
        client_socket.send("SHUTDOWN".encode('utf-8'))
        return
    # Linear Velocity
    des_vel = np.array([message.speed * np.cos(message.dir), message.speed * np.sin(message.dir)])
    # magnitude of the projection of des_vel onto A_vel
    A_vel = np.dot(des_vel, unit_A)
    B_vel = np.dot(des_vel, unit_B)
    C_vel = np.dot(des_vel, unit_C)
    # convert to rpm
    A_rpm = A_vel / wheel_diam * 60
    B_rpm = B_vel / wheel_diam * 60
    C_rpm = C_vel / wheel_diam * 60

    # Rotational Velocity
    rot = message.rot_speed
    # convert to rpm
    A_rpm += rot * robot_radius / wheel_diam * 60
    B_rpm += rot * robot_radius / wheel_diam * 60
    C_rpm += rot * robot_radius / wheel_diam * 60

    # if over max rpm, scale down
    max_rpm = max(np.abs(A_rpm), np.abs(B_rpm), np.abs(C_rpm))
    if max_rpm > max_wheel_rpm:
        scale = max_wheel_rpm / max_rpm
        A_rpm *= scale
        B_rpm *= scale
        C_rpm *= scale

    # convert to power level
    A_pwr = int(A_rpm / max_wheel_rpm * 100)
    B_pwr = int(B_rpm / max_wheel_rpm * 100)
    C_pwr = int(C_rpm / max_wheel_rpm * 100)

    # send to motors
    command = "MOVE " + str(A_pwr) + " " + str(B_pwr) + " " + str(C_pwr) + " "
    client_socket.send(command.encode('utf-8'))
    return

def listener():
    rospy.Subscriber("cmd_vel", ctrl_vec, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('kiwibot_controller', anonymous=True)
    listener()
