#!/usr/bin/env python

#before running this node you have to run the following command:
#   rosrun usb_cam usb_cam_node _pixel_format:=yuyv

import rospy
import numpy as np 
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

pub = rospy.Publisher("/coordinates", String, queue_size=10)

def publish_coordinates(x, y):
  msg = String()
  msg.data = "Coordinates: (" + str(x) + ", " + str(y) + ")"
  global pub
  pub.publish(msg)

def filter_color(rgb_image, lower_bound_color, upper_bound_color, show=False):
  #convert the image into the HSV color space
  hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
  if show:
    cv2.imshow("HSV Image", hsv_image)

  #define a mask using the upper and lower bounds of the yellow color
  mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

  return mask

def getContours(binary_image):
  (_, contours, _) = cv2.findContours(binary_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  return contours

def get_contour_center(contour):
  M = cv2.moments(contour)
  cx = -1
  cy = -1
  if (M['m00']!=0):
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])
    publish_coordinates(cx, cy)
  return cx, cy

def draw_ball_contour(rgb_image, contours):
  #black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3], 'uint8')

  for c in contours:
    area = cv2.contourArea(c)
    perimeter = cv2.arcLength(c, True)
    ((x, y), radius) = cv2.minEnclosingCircle(c)
    if (area > 3000):
      cv2.drawContours(rgb_image, [c], -1, (255,0,0), 2)
      #cv2.drawContours(black_image, [c], -1, (255,0,0), 1)
      cx, cy = get_contour_center(c)
      cv2.circle(rgb_image, (cx,cy), 3, (0,0,255), -1)
      #cv2.circle(black_image, (cx,cy), 3, (0,0,255), -1)
      #cv2.imshow("RGB Image Contours", rgb_image)
      #cv2.imshow("Black Image Contours", black_image)

def detect_ball_in_frame(image_frame):
  yellowLower = (30, 75, 75)
  yellowUpper = (60, 255, 255)
  binary_image_mask = filter_color(image_frame, (30, 75, 75), (60, 255, 255))
  contours = getContours(binary_image_mask)
  draw_ball_contour(image_frame, contours)
  cv2.imshow("Frame", image_frame)

bridge = CvBridge()

def image_callback(ros_image):
  global bridge
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  detect_ball_in_frame(cv_image)
  cv2.imshow("camera", cv_image)
  cv2.waitKey(3)

  

rospy.init_node('image_converter', anonymous=True)

image_sub = rospy.Subscriber("/usb_cam/image_raw",Image, image_callback)
try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
  cv2.destroyAllWindows()

