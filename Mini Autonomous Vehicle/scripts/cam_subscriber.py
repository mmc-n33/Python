#!/usr/bin/env python
import rospy
from io import BytesIO
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import sys
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class image_converter:

	def img_subscriber():
   #Receives images from the ROS topic that sends images#
	    img_sub = rospy.Subscriber("pi_image_topic", Image, callback)
	    bridge = CvBridge()
	    rospy.init_node("img_subscriber", anonymous=True) # Create subscriber node
	    print("Running image  subscriber")
    # Subscribe to "pi_imagw_topic"
    # callback() is called when a message is received

	def callback(data):
#Callback function of subscribed topic. Here images get converted and features detected#
  	 rospy.loginfo(rospy.get_caller_id() + " Image received: %s", data)
    
	try:
   	  img_sub.subscribe(CvBridge.cv2_to_imgmsg(pi_image, "bgr8"))
	except CvBridgeError as e:
    		print(e)	
	(rows, cols, channels) = cv_image.shape
	if cols > 60 and rows > 60:
		cv2.circle(cv_image, (150, 150), 20, (30, 144, 255), -1)
	cv2.imshow("Image window", cv_image)
	cv2.waitKey(3)
  

if __name__ == '__main__':
  ic = image_converter() 
  try:
# Keeps Python from exiting until node is stopped
   	 rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



