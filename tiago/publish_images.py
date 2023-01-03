import cv2
import os
import phase.pyphase as phase


# Define information about the Titania camera
# Each camera has unique camera_name, left_serial, and right_serial
camera_name = "746974616e24321"
left_serial = "40146993"
right_serial = "40146996"
device_type = phase.stereocamera.CameraDeviceType.DEVICE_TYPE_TITANIA
interface_type = phase.stereocamera.CamerteaInrfaceType.INTERFACE_TYPE_USB

# Define calibration files
script_path = os.path.dirname(os.path.realpath(__file__))
test_folder = script_path
data_folder = os.path.join(script_path, "test", "data")
left_yaml = os.path.join(data_folder, "titania_left.yaml")
right_yaml = os.path.join(data_folder, "titania_right.yaml")
out_ply = os.path.join(test_folder, "titania_out.ply")

# Define parameters for read process
downsample_factor = 1.0
display_downsample = 0.25
exposure_value = 20000

# Check for I3DRSGM license
license_valid = phase.stereomatcher.StereoI3DRSGM().isLicenseValid()
if license_valid:
    print("I3DRSGM license accepted")
    stereo_params = phase.stereomatcher.StereoParams(
        phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_I3DRSGM,
        9, 0, 49, False
    )
else:
    print("Missing or invalid I3DRSGM license. Will use StereoBM")
    stereo_params = phase.stereomatcher.StereoParams(
        phase.stereomatcher.StereoMatcherType.STEREO_MATCHER_BM,
        11, 0, 25, False
    )

# Load calibration
calibration = phase.calib.StereoCameraCalibration.calibrationFromYAML(left_yaml, right_yaml)

# Create stereo matcher
matcher = phase.stereomatcher.createStereoMatcher(stereo_params)

# Create stereo camera device information from parameters
device_info = phase.stereocamera.CameraDeviceInfo(
    left_serial, right_serial, camera_name,
    device_type,
    interface_type)

# Create stereo camera
tinaniaCam = phase.stereocamera.TitaniaStereoCamera(device_info)

# Connect camera and start data capture
print("Connecting to camera...")
ret = tinaniaCam.connect()

if (ret):
    tinaniaCam.startCapture()
    # Set camera exposure value
    tinaniaCam.setExposure(exposure_value)
    print("Running camera capture...")
    while tinaniaCam.isConnected():
        read_result = tinaniaCam.read()
        if read_result.valid:
            # Rectify stereo image pair
            rect_image_pair = calibration.rectify(read_result.left, read_result.right)
            rect_img_left = rect_image_pair.left
            rect_img_right = rect_image_pair.right

            # Stereo Matching
            match_result = matcher.compute(rect_img_left, rect_img_right)

            # Check compute is valid
            if not match_result.valid:
                print("Failed to compute match")
                continue

            # Find the disparity from matcher
            disparity = match_result.disparity