import rospy
import time
import apriltag
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class april_detector:
	
	def __init__(self):
		self.detector = apriltag.Detector()
		self.bridge = CvBridge()
		
		rospy.init_node('april_detector')
		
		self.image_sub_l = rospy.Subscriber('/stereo_cams/left/image_raw', Image, self.image_callback_l)
		self.image_sub_r = rospy.Subscriber('/stereo_cams/right/image_raw', Image, self.image_callback_r)
		
		self.cv_image_l = None
		self.cv_image_r = None

		
	def image_callback_l(self, msg):
		try:
			self.cv_image_l = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
			
	def image_callback_r(self, msg):
		try:
			self.cv_image_r = self.bridge.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
			
	def loop(self):
		while not rospy.is_shutdown():
			if self.cv_image_l != None:
				cv2.imshow("Left", self.cv_image_l)
			if self.cv_image_r != None:
 				cv2.imshow("Right", self.cv_image_r)
			cv2.waitKey(1)
  			print("running")

if __name__ == "__main__":
	main = april_detector()
	main.loop()
