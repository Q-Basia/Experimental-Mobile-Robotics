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
from duckietown_msgs.msg import LEDPattern, Twist2DStamped
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
        self.camera_sub = rospy.Subscriber(self.image_topic, CompressedImage, self.camera_callback)

        # publishers for LEDs, augmented image and kinematics
        self.led_pub = rospy.Publisher(f"/{self._vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=10)
        self.aug_img_pub = rospy.Publisher(f"/{self._vehicle_name}/apriltag_detector_node/augmented/compressed", CompressedImage, queue_size=1)
        self.cmd_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)
        self.v = 0.2
        self.omega = 0.0
        self.start = False
        self.rate = rospy.Rate(20)
        rospy.on_shutdown(self.shutdown_hook)

        # call navigation control node
        self.tag_pub = rospy.Publisher(f"/{self._vehicle_name}/apriltag_detector_node/tag_id", Int32, queue_size=1)

        # Initialize color detection parameters (in HSV format)
        self.red_lower = np.array([0, 95, 108], np.uint8)
        self.red_upper = np.array([8, 171, 255], np.uint8)
        homography = [
        -4.99621433668091e-05, 0.0012704090688819693, 0.2428235605203261,
        -0.001999628080487182, -5.849807527639727e-05, 0.6400119336043912,
        0.0003409556379103712, 0.0174415825291776, -3.2316507961510252
        ]
        self.H = np.array(homography).reshape((3,3))

        # Camera calibration
        self.camera_matrix = None
        self.dist_coeffs = None
        self.mapx, self.mapy = None, None
        self.got_camera_info = False

        # Define other variables
        self.last_detected_tag_type = None
        self.last_detection_time = rospy.get_time()
        self.stop_durations = {
            "stop_sign": 3.0,     # Red LED
            "t_intersection": 2.0, # Blue LED
            "uofa_tag": 1.0,      # Green LED
            "none": 0.5,      # No tag detected
        }

        # Frame counter for controlling detection rate
        self.frame_count = 0
  
        # Higher value = less frequent detection 
        self.apriltag_frame_skip = 3  # Run detection every 3 frames
        
        # Flag to track if robot is currently stopped
        self.is_stopped = False
        self.stop_start_time = None

        # initialize dt_apriltag detector (using tag36h11 family)
        self.detector = dt_apriltags.Detector(families='tag36h11')

        rospy.loginfo("ApriltagNode initialized.")

    def sign_to_led(self, tag_id):
        """
        Set the LED color based on tag ID
        """
        tag_type = None
        if tag_id == 22:
            color = [1, 0, 0, 1]  # Red - stop sign
            tag_type = "stop_sign"
        elif tag_id in [51, 133]:
            color = [0, 0, 1, 1]  # Blue - T-intersection
            tag_type = "t_intersection"
        elif tag_id == 93:
            color = [0, 1, 0, 1]  # Green - UoA Tag
            tag_type = "uofa_tag"
        else:
            tag_type = "none"
            color = [1, 1, 1, 1]  # White - default/no detection

        pattern = LEDPattern()
        for _ in range(5):
            rgba = ColorRGBA(*color)
            pattern.rgb_vals.append(rgba)

        self.led_pub.publish(pattern)
        return tag_type

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
        tag_id = None
        tag_type = "none"
        if not detections:
            self.sign_to_led(None)
            return image, tag_id, tag_type

        for det in detections:
            tag_id = det.tag_id
            center = tuple(map(int, det.center))
            corners = np.array(det.corners, dtype=np.int32)
            corner_dists = [
                np.linalg.norm(corners[0] - corners[2]),  # diagonal 1
                np.linalg.norm(corners[1] - corners[3])   # diagonal 2
            ]
            tag_size = max(corner_dists)
            if tag_size < 80: #Tag must be large i.e. close enough to consider detection
                continue

            # Draw bounding box
            cv2.polylines(image, [corners.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.putText(image, f"ID: {tag_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

            # Set LED color
            tag_type = self.sign_to_led(tag_id)

            # Just use first tag detected
            break

        return image, tag_id, tag_type
    
    def project_pixel_to_ground(self, pixel_x, pixel_y):

        point = np.array([pixel_x, pixel_y, 1.0]).reshape(3,1)
        ground_point = np.matmul(self.H, point).reshape(3)

        if abs(ground_point[2]) > 1e-7:
            ground_x = ground_point[0] / ground_point[2]
            ground_y = ground_point[1] / ground_point[2]
            return ground_x, ground_y
        else:
            rospy.logwarn("Point is too close to judge distance, zero division error")
            return None, None
        
    def detect_red_line(self, image):
        """Detect red lines in the image"""
        # Convert to HSV color space
        hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image = image[480//2:,:]
        
        # Create mask for red color
        red_mask = cv2.inRange(hsv_frame, self.red_lower, self.red_upper)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), "uint8")
        red_mask = cv2.dilate(red_mask, kernel)
        
        # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        red_line_detected = False
        
        # Check contours for significant red areas
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                x, y, w, h = cv2.boundingRect(contour)
                bottom_x, bottom_y = x + w // 2, y + h
                ground_x, ground_y = self.project_pixel_to_ground(bottom_x, bottom_y)
                # Draw bounding box on the image
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(image, f"{ground_x}m", (x, y - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                if (abs(ground_x) <= 0.25):
                    red_line_detected = True
        
        return red_line_detected, image
    
    def publish_velocity(self, v, omega):
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.cmd_pub.publish(msg)

    def stop(self, duration):
        rospy.loginfo(f"[STOP] Stopping for {duration} seconds.")
        current_time = rospy.get_time()
        while rospy.get_time() - current_time < duration:
            self.publish_velocity(v=0.0, omega=0.0)
            # self.rate.sleep()
        

    def manage_stopping(self, current_detection, red_detected):

        if red_detected and current_detection in self.stop_durations and self.stop_durations[current_detection] > 0:
            stop_duration = self.stop_durations[current_detection]
            self.stop(stop_duration)

            # Ensure we keep moving until the intersection (red line no longer detected)  
            cooldown_duration = 1.5
            current_time = rospy.get_time()
            while rospy.get_time() - current_time < cooldown_duration:
                self.publish_velocity(self.v, self.omega)
                # self.rate.sleep()


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
            
        # if not self.start:
        #     self.publish_velocity(self.v, self.omega)
        #     self.start = True
        if not self.start:
            self.publish_velocity(0,0)
            self.start = True

        self.frame_count += 1

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Preprocess image
        gray, undistorted = self.process_image(cv_image)
        red_line_detected = False
        # red_line_detected, cv_image = self.detect_red_line(undistorted)
        
        tag_id = None
        
        # Detect and annotate AprilTags
        if self.frame_count % self.apriltag_frame_skip == 0:
            annotated_img, tag_id, tag_type = self.detect_tag(gray, undistorted)
            if tag_id is not None:
                self.last_detected_tag_type = tag_type
                self.last_detection_time = rospy.get_time()
            self.publish_augmented_img(annotated_img)
        
        # Display frame processing rate information (useful for debugging)
        if self.frame_count % 30 == 0:  # Update every 30 frames
            fps_text = f"AprilTag detection every {self.apriltag_frame_skip} frames"
            cv2.putText(annotated_img, fps_text, (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                  
        self.manage_stopping(self.last_detected_tag_type, red_line_detected)
        
        
    def shutdown_hook(self):
        # Publish zero velocity command when shutting down
        self.publish_velocity(0,0)
        rospy.loginfo("Robot stopped due to shutdown")

if __name__ == '__main__':
    # create the node
    node = ApriltagNode(node_name='apriltag_detector_node')
    rospy.spin()
    
