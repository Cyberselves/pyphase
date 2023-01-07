import rospy
import time
import apriltag
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class april_detector:

    def __init__(self):
        options = apriltag.DetectorOptions(families="tag36h11")
        self.detector = apriltag.Detector(options)
        self.bridge = CvBridge()

        rospy.init_node('april_detector')

        self.image_sub_l = rospy.Subscriber('/stereo_cams/left/image_raw', Image, self.image_callback_l)
        self.image_sub_r = rospy.Subscriber('/stereo_cams/right/image_raw', Image, self.image_callback_r)

        self.cv_image_l = None
        self.cv_image_r = None

        self.left_image_received = False
        self.right_image_received = False

        self.grayscale_l = None
        self.grayscale_r = None

    def image_callback_l(self, msg):
        try:
            self.cv_image_l = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.left_image_received = True
        except CvBridgeError as e:
            print(e)
            
        #self.grayscale_l = cv2.cvtColor(self.cv_image_l, cv2.COLOR_BGR2GRAY)
        #results = self.detector.detect(self.grayscale_l)
        
        #self.draw_tags(self.cv_image_l, results)
        
    def image_callback_r(self, msg):
        try:
            self.cv_image_r = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.right_image_received = True
        except CvBridgeError as e:
            print(e)
            
        self.grayscale_r = cv2.cvtColor(self.cv_image_r, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(self.grayscale_r)
        
        self.draw_tags(self.cv_image_r, results)
        
        pose = self.detector.detection_pose(results, self.camera_params_r, tag_size=1, z_sign=1)
        
    def draw_tags(self, img, tags):
        # loop over the AprilTag detection results
        for tag in tags:
                # extract the bounding box (x, y)-coordinates for the AprilTag
                # and convert each of the (x, y)-coordinate pairs to integers
                (ptA, ptB, ptC, ptD) = tag.corners
                ptB = (int(ptB[0]), int(ptB[1]))
                ptC = (int(ptC[0]), int(ptC[1]))
                ptD = (int(ptD[0]), int(ptD[1]))
                ptA = (int(ptA[0]), int(ptA[1]))
                
                # draw the bounding box of the AprilTag detection
                cv2.line(img, ptA, ptB, (0, 255, 0), 2)
                cv2.line(img, ptB, ptC, (0, 255, 0), 2)
                cv2.line(img, ptC, ptD, (0, 255, 0), 2)
                cv2.line(img, ptD, ptA, (0, 255, 0), 2)
                
                # draw the center (x, y)-coordinates of the AprilTag
                (cX, cY) = (int(tag.center[0]), int(tag.center[1]))
                cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)


    def loop(self):
        while not rospy.is_shutdown():
            if self.left_image_received:
                cv2.imshow("Left", self.cv_image_l)
            if self.right_image_received:
                cv2.imshow("Right", self.cv_image_r)
            cv2.waitKey(1)

if __name__ == "__main__":
    main = april_detector()
    main.loop()
