import rospy
import time
import apriltag
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

detector = apriltag.Detector()
bridge = CvBridge()

def image_callback_l(msg):
  try:
    cv_image_l = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError as e:
    print(e)

def image_callback_r(msg):
  try:
    cv_image_r = bridge.imgmsg_to_cv2(msg, "bgr8")
  except CvBridgeError as e:
    print(e)

rospy.init_node('april_detector')

image_sub_l = rospy.Subscriber('/stereo_cams/left/image_raw', Image, image_callback_l)
image_sub_r = rospy.Subscriber('/stereo_cams/right/image_raw', Image, image_callback_r)

while not rospy.is_shutdown():
  cv2.imshow("Left", cv_image_l)
  cv2.waitKey(1)
  cv2.imshow("Right", cv_image_r)
  cv2.waitKey(1)
  print("running")
  time.sleep(10)
