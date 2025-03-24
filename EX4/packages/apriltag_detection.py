#!/usr/bin/env python3

# potentially useful for part 1 of exercise 4

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType
import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import ColorRGBA
from duckietown_msgs.msg import LEDPattern
import numpy as np
from std_msgs.msg import Int32
import dt_apriltags

class ApriltagNode(DTROS):

    def __init__(self, node_name):
        super(ApriltagNode, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)

        # add your code here
        # initialize vehicle name 
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.bridge = CvBridge()

        # subscribe to camera feed
        self.image_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"
        rospy.Subscriber(self.image_topic, CompressedImage, self.camera_callback)
        # rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)

        # publishers for LEDs and augmented image
        self.led_pub = rospy.Publisher(f"/{self._vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=10)
        self.aug_img_pub = rospy.Publisher(f"/{self._vehicle_name}/apriltag_detector_node/augmented/compressed", CompressedImage, queue_size=1)
        
        # call navigation control node
        self.tag_pub = rospy.Publisher(f"/{self._vehicle_name}/apriltag_detector_node/tag_id", Int32, queue_size=1)

        # define other variables as needed

        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.mapx, self.mapy = None, None
        self.got_camera_info = False

        # initialize dt_apriltag detector (using tag36h11 family)
        self.detector = dt_apriltags.Detector(families='tag36h11')

        rospy.loginfo("ApriltagNode initialized.")

    def sign_to_led(self, tag_id):
        """
        Set the LED color based on tag ID
        """
        if tag_id == 0:
            color = [1, 0, 0, 1]  # Red - stop sign
        elif tag_id == 1:
            color = [0, 0, 1, 1]  # Blue - T-intersection
        elif tag_id == 2:
            color = [0, 1, 0, 1]  # Green - UoA Tag
        else:
            color = [1, 1, 1, 1]  # White - default/no detection

        pattern = LEDPattern()
        for _ in range(5):
            rgba = ColorRGBA(*color)
            pattern.rgb_vals.append(rgba)

        self.led_pub.publish(pattern)

    def process_image(self, image):
        """
        Preprocess image: undistort and convert to grayscale
        """
        if self.got_camera_info:
            undistorted = cv2.remap(image, self.mapx, self.mapy, cv2.INTER_LINEAR) 
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        return gray, undistorted

    def publish_augmented_img(self, image):
        """
        Publish image with bounding boxes and tag IDs
        """
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
        self.aug_img_pub.publish(msg)

    def publish_leds(self, **kwargs):
        """
        Publish the current LED color
        """
        pattern = LEDPattern()
        for _ in range(5):
            rgba = ColorRGBA(*self.current_led_color)
            pattern.rgb_vals.append(rgba)
        self.led_pub.publish(pattern)

    def detect_tag(self, gray, image):
        """
        Detect tags in the grayscale image and annotate them on the image
        """
        detections = self.detector.detect(gray)
        if not detections:
            self.sign_to_led(None)
            return image

        for det in detections:
            tag_id = det.tag_id
            center = tuple(map(int, det.center))
            corners = np.array(det.corners, dtype=np.int32)

            # Draw bounding box
            cv2.polylines(image, [corners.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.putText(image, f"ID: {tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

            # Set LED color
            self.sign_to_led(tag_id)

            # Just use first tag detected
            break

        return image

    def camera_callback(self, msg):
        """
        Convert compressed image to CV2 image, process and detect AprilTags
        """
        if not self.got_camera_info:
            try:
                cam_info = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=5.0)
                self.camera_matrix = np.array(cam_info.K).reshape((3, 3))
                self.dist_coeffs = np.array(cam_info.D)
                h, w = 480, 640
                new_K, _ = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w, h), 1, (w, h))
                self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, None, new_K, (w, h), cv2.CV_32FC1)
                self.got_camera_info = True
                rospy.loginfo("Got camera calibration info.")
            except:
                rospy.logwarn("CameraInfo not received yet.")
                return

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Preprocess image
        gray, undistorted = self.process_image(cv_image)

        # Detect and annotate AprilTags
        annotated_img = self.detect_tag(gray, undistorted)

        # Publish annotated image
        self.publish_augmented_img(annotated_img)


if __name__ == '__main__':
    # create the node
    node = ApriltagNode(node_name='apriltag_detector_node')
    rospy.spin()
    
