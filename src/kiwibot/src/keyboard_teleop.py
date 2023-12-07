#!/usr/bin/env python
import rospy
import numpy as np
from pynput import keyboard
from time import sleep
from kiwibot.msg import ctrl_vec
from notify_run import Notify
notify = Notify()

current_keys = set()
def on_press(key):
    current_keys.add(key)
def on_release(key):
    current_keys.remove(key)
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

def teleop():
    print("I, J, K, L for movement")
    print("A for CW rotation, D for CCW rotation")
    print("W to increase throttle, S to decrease")
    print("Q to close connection, E to shutdown robot")
    pub = rospy.Publisher('cmd_vel', ctrl_vec, queue_size=10)
    rate = rospy.Rate(30)
    throttle = 1.0
    while not rospy.is_shutdown():
        # I, J, K, L for movement
        # A for CW rotation, D for CCW rotation
        # W to increase throttle, S to decrease
        # Q for quit
        # Simultaneous key presses are supported
        translation = np.array([0.0, 0.0])
        rotation = 0.0

        if keyboard.KeyCode.from_char('i') in current_keys:
            translation[1] += 1.0
        if keyboard.KeyCode.from_char('k') in current_keys:
            translation[1] -= 1.0
        if keyboard.KeyCode.from_char('j') in current_keys:
            translation[0] += 1.0
        if keyboard.KeyCode.from_char('l') in current_keys:
            translation[0] -= 1.0
        if keyboard.KeyCode.from_char('a') in current_keys:
            rotation += 3.0
        if keyboard.KeyCode.from_char('d') in current_keys:
            rotation -= 3.0
        if keyboard.KeyCode.from_char('w') in current_keys:
            throttle += 0.1
            if throttle < 0.0:
                throttle = 0.0
            if throttle > 1.0:
                throttle = 1.0
            print("Throttle:", throttle)
            # notify.send("Throttle: " + str(throttle))
            rate.sleep()
            rate.sleep()
            rate.sleep()
            rate.sleep()
        if keyboard.KeyCode.from_char('s') in current_keys:
            throttle -= 0.1
            if throttle < 0.0:
                throttle = 0.0
            if throttle > 1.0:
                throttle = 1.0
            print("Throttle:", throttle)
            # notify.send("Throttle: " + str(throttle))
            rate.sleep()
            rate.sleep()
            rate.sleep()
            rate.sleep()
        if keyboard.KeyCode.from_char('q') in current_keys:
            msg = ctrl_vec()
            msg.cmd = "quit"
            pub.publish(msg)
            break
        if keyboard.KeyCode.from_char('e') in current_keys:
            msg = ctrl_vec()
            msg.cmd = "stop"
            pub.publish(msg)
            break

        # Normalize translation vector
        if np.linalg.norm(translation) > 0.0:
            translation = translation / np.linalg.norm(translation)
        
        # Scale translation vector by throttle
        translation *= throttle
        # Scale rotation by throttle
        rotation *= throttle
        
        msg = ctrl_vec()
        msg.speed = np.linalg.norm(translation)
        msg.dir = np.arctan2(translation[1], translation[0])
        msg.rot_speed = rotation
        pub.publish(msg)
        # print("Published", msg)
        rate.sleep()
    


if __name__ == '__main__':
    rospy.init_node('keyboard_teleop', anonymous=True)
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass