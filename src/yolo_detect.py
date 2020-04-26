#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('yolo_ros_detect')
import sys
import rospy
import cv2 as cv
import numpy as np
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class yolo_detect:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/camera/rgb/image_rect_color',Image,self.callback)    
    self.pub = rospy.Publisher('/yolo_ros_detect/image', Image, queue_size=1)
    #class names
    f = open('/home/bloisi/catkin_ws/src/yolo_ros_detect/model/coco.names', 'r')
    self.labels = []
    for line in f:
        self.labels.append(line.strip())
    self.labels = np.array(self.labels)
    # generate different colors for different classes 
    self.colors = np.random.uniform(0, 255, size=(len(self.labels), 3))
    #init the network
    self.net = cv.dnn.readNetFromDarknet(
                      '/home/bloisi/catkin_ws/src/yolo_ros_detect/model/yolov2.cfg',
                      '/home/bloisi/catkin_ws/src/yolo_ros_detect/model/yolov2.weights')


  def callback(self, data):
    
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
           
    except CvBridgeError as e:
      print(e)

    img_height = img.shape[0]
    img_width = img.shape[1]

    #feed the image into the network

    resized_img = cv.resize(img, (416, 416))
    inputBlob = cv.dnn.blobFromImage(resized_img, 1.0/255.0)

    self.net.setInput(inputBlob, 'data')

    #feed forward the model
    result = self.net.forward()

    #thresholding results 
    for i in range(result.shape[0]):
        probability_index = 5
        probability_size = result.shape[1] - probability_index
            
        objectClass = np.argmax(result[i][probability_index:])
            
        confidence = result[i][probability_index + objectClass]

        if confidence > 0.3:
            (x, y, width, height) = result[i][0:3+1]
                
            xLeftBottom = (x - (width / 2.0)) * img_width
            yLeftBottom = (y - (height / 2.0)) * img_height
            xRightTop = (x + (width / 2.0)) * img_width
            yRighttop = (y + (height / 2.0)) * img_height
                
            if objectClass < self.labels.shape[0]:
                label_to_put = self.labels[objectClass] #+ str(confidence)
                pixelLeftBottom = (int(xLeftBottom), int(yLeftBottom))
                pxielRightTop = (int(xRightTop), int(yRighttop))
                cv.rectangle(img, pt1=pixelLeftBottom, pt2=pxielRightTop, color=self.colors[objectClass], thickness=1)
                cv.putText(img, text=label_to_put, org=pixelLeftBottom,
                                fontFace=cv.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=self.colors[objectClass],thickness=1)


    #convert opencv format back to ros format and publish result
    try:
      detection_message = self.bridge.cv2_to_imgmsg(img, "bgr8")
      self.pub.publish(detection_message)
    except CvBridgeError as e:
      print(e)
    


def main(args):
  detector = yolo_detect()
  rospy.init_node('yolo_ros_detect', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

