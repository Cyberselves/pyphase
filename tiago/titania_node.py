# General imports
import time
import os

# OpenCV imports
import cv2
import numpy
from cv_bridge import CvBridge, CvBridgeError

# Apriltag Imports
import apriltag
from tf.transformations import quaternion_about_axis

# Phase Imports
import phase.pyphase as phase

# ROS Imports
import rospy
from sensor_msgs.msg import CompressedImage
from stereo_msgs.msg import DisparityImage
from geometry_msgs.msg import Transform, Quaternion

debug = False

class titania_node:

    def __init__(self, debug=False):

        self.debug = debug

        # Initialise Apriltag Detector
        options = apriltag.DetectorOptions(
            families="tag36h11",
            border=1,
            nthreads=4,
            quad_decimate=1.0,
            quad_blur=0.0,
            refine_edges=True,
            refine_decode=False,
            refine_pose=True,
            debug=False,
            quad_contours=True)
        self.tag_size = 0.0635
        self.z_sign = 1
        self.detector = apriltag.Detector(options)
        self.april_transform_l = Transform()
        self.april_transform_r = Transform()

        # Initialise ROS
        rospy.init_node("Titania")
        self.image_l_pub = rospy.Publisher('titania/left/image_raw/compressed', CompressedImage, queue_size=1)
        self.image_r_pub = rospy.Publisher('titania/right/image_raw/compressed', CompressedImage, queue_size=1)
        self.image_disp_pub = rospy.Publisher('titania/depth/image_raw', DisparityImage, queue_size=1)
        self.image_disp_pub_comp = rospy.Publisher('titania/depth/image_raw/compressed', CompressedImage, queue_size=1)
        self.left_transform_pub = rospy.Publisher("titania/left/april_transform", Transform, queue_size=1)
        self.right_transform_pub = rospy.Publisher("titania/right/april_transform", Transform, queue_size=1)

        # Image Conversion Setup
        self.bridge = CvBridge()

        # Initialise Titania
        # Define information about the Titania camera
        # Each camera has unique camera_name, left_serial, and right_serial
        self.camera_name = "746974616e24321"
        self.left_serial = "40146993"
        self.right_serial = "40146996"
        self.device_type = phase.stereocamera.CameraDeviceType.DEVICE_TYPE_TITANIA
        self.interface_type = phase.stereocamera.CameraInterfaceType.INTERFACE_TYPE_USB

        # Define calibration files
        script_path = os.path.dirname(os.path.realpath(__file__))
        test_folder = os.path.join(script_path, "..", "test")
        data_folder = os.path.join(test_folder, "data")
        self.left_yaml = os.path.join(data_folder, "titania_left.yaml")
        self.right_yaml = os.path.join(data_folder, "titania_right.yaml")

        # Define parameters for read process
        self.downsample_factor = 1.0
        self.display_downsample = 1.0
        self.exposure_value = 20000

        # Check for I3DRSGM license
        license_valid = phase.stereomatcher.StereoI3DRSGM().isLicenseValid()
        if license_valid:
            print("I3DRSGM license accepted")
            self.stereo_params = phase.stereomatcher.StereoParams(
                phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_I3DRSGM,
                9, 0, 49, False
            )
        else:
            print("Missing or invalid I3DRSGM license. Will use StereoBM")
            self.stereo_params = phase.stereomatcher.StereoParams(
                phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_BM,
                11, 0, 25, False
            )

        # Load calibration
        self.calibration = phase.calib.StereoCameraCalibration.calibrationFromYAML(self.left_yaml, self.right_yaml)

        # Create stereo matcher
        self.matcher = phase.stereomatcher.createStereoMatcher(self.stereo_params)

        # Create stereo camera device information from parameters
        self.device_info = phase.stereocamera.CameraDeviceInfo(
            self.left_serial, self.right_serial, self.camera_name,
            self.device_type,
            self.interface_type)

        # Create stereo camera
        self.titaniaCam = phase.stereocamera.TitaniaStereoCamera(self.device_info)

        # Set Fx Fy Cx Cy for cameras
        self.camera_params_l = (1744.9, 1745.47, 931.505, 581.185)
        self.camera_params_r = (1744.12, 1744.4, 962.712, 587.003)

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

        # Connect camera and start data capture
        print("Connecting to camera...")
        connected = self.titaniaCam.connect()

        if (connected):
            # Start capturing images and set exposure value
            self.titaniaCam.startCapture()
            self.titaniaCam.setExposure(self.exposure_value)

            #Start Loop
            while not rospy.is_shutdown() and self.titaniaCam.isConnected:
                read_result = self.titaniaCam.read()
                if read_result.valid:
                    # Rectify stereo image pair
                    rect_image_pair = self.calibration.rectify(read_result.left, read_result.right)

                    # Perform Stereo Matching
                    match_result = self.matcher.compute(rect_image_pair.left, rect_image_pair.right)

                    # Get disparity from matched images
                    if match_result.valid:
                        disparity = match_result.disparity
                        disparity_image = phase.normaliseDisparity(disparity)
                        #depth_image = phase.disparity2depth(match_result.disparity, self.calibration.getQ())
                    else:
                        print("Failed to compute match, no disparity image created")
                        continue

                    """

                    # Convert Images to grayscale for apriltag detection
                    grayscale_l = cv2.cvtColor(rect_image_pair.left, cv2.COLOR_BGR2GRAY)
                    grayscale_r = cv2.cvtColor(rect_image_pair.right, cv2.COLOR_BGR2GRAY)

                    # Detect Apriltags
                    left_tags = self.detector.detect(grayscale_l)
                    right_tags = self.detector.detect(grayscale_r)

                    # Find if any tags are detected
                    if len(left_tags) > 0:
                        left_tag_found = True
                        print("Left Tag Detected")
                    else:
                        left_tag_found = False

                    if len(right_tags) > 0:
                        right_tag_found = True
                        print("Right Tag Detected")
                    else:
                        right_tag_found = False

                    if left_tag_found:
                        # Calculate pose of tag
                        left_tag_pose, l_e0, r_e0 = self.detector.detection_pose(left_tags[0], self.camera_params_l, self.tag_size, self.z_sign)

                        # Calculate rvec and tvec for tag pose
                        rvec_l, _ = cv2.Rodrigues(left_tag_pose[:3, :3])
                        tvec_l = left_tag_pose[:3, 3]
                        print(tvec_l)

                        # Convert rvec to quat and populate msg
                        self.april_transform_l.translation.x = tvec_l[0]
                        self.april_transform_l.translation.y = tvec_l[1]
                        self.april_transform_l.translation.z = tvec_l[2]
                        angle = cv2.norm(rvec_l[0])
                        self.april_transform_l.rotation = Quaternion(*quaternion_about_axis(angle, rvec_l[0]))

                    if right_tag_found:
                        # Calculate pose of tag
                        right_tag_pose, r_e0, r_e1 = self.detector.detection_pose(right_tags[0], self.camera_params_r, self.tag_size, self.z_sign)

                        # Calculate rvec and tvec for each tag pose
                        rvec_r, _ = cv2.Rodrigues(right_tag_pose[:3, :3])
                        tvec_r = right_tag_pose[:3, 3]

                        # Convert rvec to quat and populate msg
                        self.april_transform_r.translation.x = tvec_r[0]
                        self.april_transform_r.translation.y = tvec_r[1]
                        self.april_transform_r.translation.z = tvec_r[2]
                        angle = cv2.norm(rvec_r[0])
                        self.april_transform_r.rotation = Quaternion(*quaternion_about_axis(angle, rvec_r[0]))

                    """

                    # Convert opencv images to ros msgs
                    ros_image_l = self.bridge.cv2_to_compressed_imgmsg(rect_image_pair.left)
                    ros_image_r = self.bridge.cv2_to_compressed_imgmsg(rect_image_pair.right)
                    if match_result.valid:
                        ros_image_disp_comp = self.bridge.cv2_to_compressed_imgmsg(disparity_image)
                        #ros_image_disp = self.bridge.cv2_to_imgmsg(disparity, encoding="passthrough")

                    # Publish to ros
                    self.image_l_pub.publish(ros_image_l)
                    self.image_r_pub.publish(ros_image_r)
                    if match_result.valid:
                        #disparity_msg = DisparityImage()
                        #disparity_msg.image = ros_image_disp
                        #self.image_disp_pub.publish(disparity_msg)
                        print("publishing")
                        self.image_disp_pub_comp.publish(ros_image_disp_comp)

                    """
                    if left_tag_found:
                        self.left_transform_pub.publish(self.april_transform_l)
                    if right_tag_found:
                        self.right_transform_pub.publish(self.april_transform_r)
                    """

                    # Annotate and Display images only if debug is true
                    if self.debug:
                        # Copy image for annotation
                        annotate_l = rect_image_pair.left.copy()
                        annotate_r = rect_image_pair.right.copy()

                        # Draw apriltags onto image for debug
                        if left_tag_found:
                            self.draw_tags(annotate_l, left_tags)
                            self.draw_axes(annotate_l, self.camera_params_l, self.tag_size, left_tag_pose, left_tags[0].center)
                        if right_tag_found:
                            self.draw_tags(annotate_r, right_tags)
                            self.draw_axes(annotate_r, self.camera_params_r, self.tag_size, right_tag_pose, right_tags[0].center)

                        # Display Images
                        cv2.imshow("Left", annotate_l)
                        cv2.imshow("Right", annotate_r)
                        cv2.imshow("Disparity", disparity_image)
                        cv2.waitKey(1)

                else:
                    #self.titaniaCam.disconnect()
                    #raise Exception("Failed to read stereo result")
                    print("Failed to get stereo result")

        cv2.destroyAllWindows()
        self.titaniaCam.disconnect()

if __name__ == "__main__":
    main = titania_node(debug=debug)
    main.loop()
