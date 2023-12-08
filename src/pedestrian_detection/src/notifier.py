#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from notify_run import Notify
from Adafruit_IO import Client, Data

notify = Notify()
aio = Client('chpaxson', 'aio_vWwC74oKX9AhulVovIljHrLYyxKw')
print(aio.feeds())
# aio.create_feed('people_detected')

prev_notif_time = 0

# Subscriber callback function
def send_notification(confidence):
    global prev_notif_time
    human_count = len(confidence)
    if (rospy.get_time() - prev_notif_time < 20 or human_count == 0):
        return
    if human_count == 1:
        msg = "1 person detected with confidence "
    else:
        msg = str(human_count) + " people detected with confidences "
    for c in confidence:
        msg += f'{c:.3f}, '
    msg = msg[:-2]
    notify.send(msg)
    print(msg)
    # aio.create_data('people_detected', Data(value=human_count))
    aio.send_data('people', human_count)
    prev_notif_time = rospy.get_time()

def callback(message):
    confidence = []
    for i in range(len(message.data)):
        if (message.data[i] > 0.85):
            confidence.append(message.data[i])
    send_notification(confidence)
            
    
def notifier():
    rospy.Subscriber("pedestrian_detection", Float64MultiArray, callback)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('notifier', anonymous=True)
    notifier()
