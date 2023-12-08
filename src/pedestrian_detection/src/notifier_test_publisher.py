#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
            
    
def notifier():
    pub = rospy.Publisher("pedestrian_detection", Float64MultiArray, queue_size=10)
    rate = rospy.Rate(0.25)
    while not rospy.is_shutdown():
        msg = Float64MultiArray()
        msg.data.append(0.9)
        print("Publishing!")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('notifier_test_publisher', anonymous=True)
    notifier()
