#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('yolo_ros_detect')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class unibas_visualizer:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/yolo_ros_detect/image", Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)    

    cv2.imshow("detection", cv_image)
    cv2.waitKey(30)
   

def main(args):
  visualizer = unibas_visualizer()
  rospy.init_node('yolo_ros_detect', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

