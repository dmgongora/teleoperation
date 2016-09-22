#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('teleoperation')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from random import randint
from sensor_msgs.msg import Joy

noise = 0

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
    self.joy_sub = rospy.Subscriber("/joy", Joy ,self.callback2)
  
  def callback2(self, data):
    global noise
    '''
    Horizontal: data.axes[0]
    Vertical:   data.axes[1]
    '''
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.axes[1])
    noise = int(40*(0.1*abs(data.axes[0]) + 0.1*abs(data.axes[1])))
    rospy.loginfo("Noise %s", noise)
    #noise_x = 
    #noise_y = abs(int(100*data.axes[1]))


  def callback(self,data):
    global noise
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, 
                 center = (160,60), 
                 radius = 10, 
                 color = 0, 
                 thickness = 5) 
      '''
      cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
      '''
    # Crop image
    cv_image = cv_image[0.5*rows-200:0.5*rows+200, 0.5*cols-200:0.5*cols+200]
    # Gray scale
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    noise_x = randint(int(-0.1*noise), int(0.1*noise))
    noise_y = randint(-noise, noise)
    # Add noise
    M = np.float32([[1,0,noise_x],[0,1,noise_y]])
    #dst = cv2.warpAffine(gray_image, M, (cols,rows))
    dst = cv2.warpAffine(gray_image, M, (400,400))
    # Enable fullscreen
    cv2.namedWindow("test", cv2.WND_PROP_FULLSCREEN)  
    cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
    cv2.imshow("test", dst)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

def joy_listener():
#  rospy.init_node('joy_listener', anonymous=True)
 # rospy.Subscriber("joy", String, joy_callback)
  rospy.spin()  


if __name__ == '__main__':
#    joy_listener()
    main(sys.argv)
