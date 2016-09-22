#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('teleoperation')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Joy

class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2", CompressedImage)
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, self.callback, queue_size = 1)
    self.joy_sub = rospy.Subscriber("/joy", Joy ,self.joyCallback)
    self.black_screen = True

  def callback(self, data):
    np_arr = np.fromstring(data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    image_height, image_width = image_np.shape[:2]
    # Crop image
    image_np = image_np[0.5*image_height-120:0.5*image_height+150, 0.5*image_width-160:0.5*image_width+160]

    # Enable fullscreen
    cv2.namedWindow("test", cv2.WND_PROP_FULLSCREEN)  
    cv2.setWindowProperty("test", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)

    # Draw lines
    image_height, image_width = image_np.shape[:2]
    center_x = int(0.5*image_width)
    center_y = int(0.5*image_height)
    '''
    cv2.circle(image_np, 
               center = (center_x, center_y), 
               radius = 10, 
               color = 0, 
               thickness = 5) 
    '''
    cv2.line(image_np, (center_x, center_y+10), (center_x, center_y-10), color = 255)
    cv2.line(image_np, (center_x+10, center_y), (center_x-10, center_y), color = 255)
    #cv2.rectangle(image_np, (center_x-140, center_y-120), (center_x+140, center_y+120), color = 255, thickness = 1)
    cv2.rectangle(image_np, (center_x-60, center_y-60), (center_x+60, center_y+60), color = 255, thickness = 1)
    

    if (self.black_screen):
      image_np = np.zeros_like(image_np)
    cv2.imshow("test", image_np)
    cv2.waitKey(3)

    # Create compressedImage #
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    self.image_pub.publish(msg)
    
  def joyCallback(self, data):
    if(data.buttons[1]):
      self.black_screen = not(self.black_screen)
      rospy.loginfo(self.black_screen)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
