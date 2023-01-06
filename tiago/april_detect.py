import rospy
from sensor_msgs.msg import Image

def image_callback_l(msg):
  print("left image recieved")

def image_callback_r(msg):
  print("right image recieved")

rospy.init_node('april_detector')

image_sub_l = rospy.Subscriber('/stereo_cams/left/image_raw', Image, image_callback_l)
image_sub_r = rospy.Subscriber('/stereo_cams/right/image_raw', Image, image_callback_r)

rospy.spin()