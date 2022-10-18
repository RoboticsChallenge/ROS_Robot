#!/usr/bin/env python
from queue import Empty
from attr import NOTHING
import roslib
roslib.load_manifest('ros_robot_description')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("ros_robot/camera_front/image_raw",Image,self.callback)
    #self.makeTrackBars() #for debugging HSV
    
  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      self.imageProccessing(cv_image)
    except CvBridgeError as e:
      print(e)
  def empty(self):
    pass
  def makeTrackBars(self):
    #makes trackbars called TrackBars
    cv2.namedWindow("TrackBars")
    cv2.resizeWindow("TrackBar",640,240)
    cv2.createTrackbar("Hue Min","TrackBars",0,179, self.empty)
    cv2.createTrackbar("Hue Max","TrackBars",179,179,self.empty)
    cv2.createTrackbar("Sat Min","TrackBars",0,255,self.empty)
    cv2.createTrackbar("Sat Max","TrackBars",255,255,self.empty)
    cv2.createTrackbar("Val Min","TrackBars",0,255,self.empty)
    cv2.createTrackbar("Val Max","TrackBars",255,255,self.empty)    
  def hsvTweaker(self,image):
      #assigns trackbars til mask
      h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")
      h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")
      s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")
      s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")
      v_min = cv2.getTrackbarPos("Val Min", "TrackBars")
      v_max = cv2.getTrackbarPos("Val Max", "TrackBars")
      print(h_min,h_max,s_min,s_max,v_min,v_max)
      lower = np.array([h_min,s_min,v_min])
      upper = np.array([h_max,s_max,v_max])
      mask = cv2.inRange(image,lower,upper)
      return mask
  def imageProccessing(self,image):
     # lower boundary RED color range values; Hue (0 - 10)
      lower1 = np.array([93, 0, 96])
      upper1 = np.array([117, 255, 255]) # when picking hsv values from 0-360, we need 0-180
      #image =cv2.GaussianBlur(image,(7,7),1) #no effect 
      hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, lower1, upper1)
      mask = cv2.bitwise_not(mask)
      contours,hierarchy = cv2.findContours(mask, 1, cv2.CHAIN_APPROX_SIMPLE) #detects all shapes
      if len(contours) > 0:
          c = max(contours, key=cv2.contourArea)
          M = cv2.moments(c)  #magic conversion of image to mystical beeing
          if M['m00'] != 0:
              cx = int(M['m10']/M['m00']) #finds center of mystical beeing
              cy = int(M['m01']/M['m00']) #finds center of mystical beeing
              cv2.circle(image, (cx, cy), 7, (255, 255, 255), -1)
              cv2.drawContours(image, c, -1, (0,255,0), 1)

      #Hori = np.concatenate((image, result), axis=1)
      #x = self.hsvTweaker(hsv) #for HSV debugging
      #cv2.imshow("Image w", imageblur)
      cv2.imshow("Image window", image)
      cv2.waitKey(3)

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