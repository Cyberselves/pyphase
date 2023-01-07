import rospy
import time
import apriltag
import cv2
import numpy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class april_detector:

    def __init__(self):
        options = apriltag.DetectorOptions(families="tag36h11",
                                           border=1,
                                           nthreads=4,
                                           quad_decimate=1.0,
                                           quad_blur=0.0,
                                           refine_edges=True,
                                           refine_decode=False,
                                           refine_pose=False,
                                           debug=False,
                                           quad_contours=True)
        
        self.tag_size=1
        self.z_sign=1
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

        # This is Gazebo Stereo Params
        self.camera_params_l = (56.424106434582825, 56.424106434582825, 320.5, 240.5)
        self.camera_params_r = (56.424106434582825, 56.424106434582825, 320.5, 240.5)

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
        
        pose, e0, e1 = self.detector.detection_pose(results[0], self.camera_params_r, self.tag_size, self.z_sign)
        print(pose)
        
        #self.draw_box(self.cv_image_r, self.camera_params_r, pose)
        self.draw_axes(self.cv_image_r, self.camera_params_r, self.tag_size, pose, results[0].center)

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
                
    def draw_box(self, img, camera_params, tag_pose):
        opoints = numpy.array([-1, -1, 0,
                                1, -1, 0,
                                1,  1, 0,
                                -1,  1, 0,
                                -1, -1, -2*self.z_sign,
                                1, -1, -2*self.z_sign,
                                1,  1, -2*self.z_sign,
                                -1,  1, -2*self.z_sign,
        ]).reshape(-1, 1, 3) * 0.5*self.tag_size

        edges = numpy.array([0, 1, 1, 2, 2, 3, 3, 0,
                             0, 4, 1, 5, 2, 6, 3, 7,
                             4, 5, 5, 6, 6, 7, 7, 4
        ]).reshape(-1, 2)

        fx, fy, cx, cy = camera_params

        K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

        rvec, _ = cv2.Rodrigues(tag_pose[:3,:3])
        tvec = tag_pose[:3, 3]

        dcoeffs = numpy.zeros(5)

        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

        ipoints = numpy.round(ipoints).astype(int)

        ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

        for i, j in edges:
                cv2.line(img, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)
                
    def draw_axes(self, img, camera_params, tag_size, tag_pose, center):
    
        fx, fy, cx, cy = camera_params
        K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

        rvec, _ = cv2.Rodrigues(tag_pose[:3,:3])
        tvec = tag_pose[:3, 3]

        dcoeffs = numpy.zeros(5)

        opoints = numpy.float32([[1,0,0],
                                 [0,-1,0],
                                 [0,0,-1]]).reshape(-1,3) * tag_size

        ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
        ipoints = numpy.round(ipoints).astype(int)

        center = numpy.round(center).astype(int)
        center = tuple(center.ravel())

        cv2.line(img, center, tuple(ipoints[0].ravel()), (0,0,255), 2)
        cv2.line(img, center, tuple(ipoints[1].ravel()), (0,255,0), 2)
        cv2.line(img, center, tuple(ipoints[2].ravel()), (255,0,0), 2)

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
