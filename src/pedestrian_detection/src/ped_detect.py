#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest("pedestrian_detection")
import sys, rospy, cv2
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class image_converter:
    
    def __init__(self):
        self.pub = rospy.Publisher("pedestrian_detection", Float64MultiArray, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("cam/image_raw", Image, self.callback)

        self.model = YOLO("yolov8n.pt")
        self.model.fuse()


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        results = self.model.predict(source=cv_image, stream=False, device="cpu", show=True)
        msg = Float64MultiArray()
        for r in results:
            for i in range(len(r.boxes.cls)):
                if (r.boxes.cls[i] == 0):
                    print("Person detected with confidence: ", r.boxes.conf[i])
                    msg.data.append(r.boxes.conf[i])
        self.pub.publish(msg)
        cv2.waitKey(3)

def main(args):
    ic = image_converter()
    rospy.init_node("image_converter", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)