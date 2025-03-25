#!/usr/bin/env python3

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import Bool, Float32
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
import time

class SafeNavigationNode(DTROS):

    def __init__(self, node_name):
        super(SafeNavigationNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # Publishers
        self.obstacle_pub = rospy.Publisher(
            f"/{rospy.get_namespace()}obstacle_detected", Bool, queue_size=1
        )
        
        # Publisher for motor control
        self.car_cmd_pub = rospy.Publisher(
            f"/{rospy.get_namespace()}car_cmd_switch_node/cmd", Twist2DStamped, queue_size=1
        )

        # Subscribers
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            f"/{rospy.get_namespace()}camera_node/image/compressed", CompressedImage, self.image_callback
        )

        # State variables
        self.obstacle_detected = False
        self.maneuvering = False
        self.maneuver_step = 0
        self.maneuver_start_time = 0
        
        # Lane following parameters
        self.lane_p_gain = 0.5  # P control gain
        self.base_speed = 0.3   # Normal speed when no obstacle
        self.maneuver_speed = 0.2  # Slower speed during maneuvers
        
        # Image processing variables
        self.last_error = 0
        self.mask_yellow = None
        self.mask_white = None
        
        # ROI for lane detection (bottom part of the image)
        self.roi_percentage = 0.3  # Use bottom 30% of image
        
        rospy.loginfo("SafeNavigationNode initialized.")

    def detect_bot(self, img):
        """
        Detects another Duckiebot in the way.
        Uses blue color filtering (since Duckiebots have blue bodies).
        """
        # Convert image to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define the blue color range (Duckiebot body)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([130, 255, 255])
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours of blue objects
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            # Calculate centroid of contour to determine position
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = 0, 0

            # Ignore small blue objects (reflections, random noise)
            if area < 1000:
                continue

            # Store obstacle position (left/right side of frame)
            img_center = img.shape[1] // 2
            self.obstacle_side = "left" if cx < img_center else "right"
            
            # If a large blue object is found, assume it's a Duckiebot
            rospy.loginfo(f"Detected a Duckiebot on the {self.obstacle_side} side!")
            return True  # Obstacle detected

        return False  # No Duckiebot detected

    def detect_lane_markers(self, img):
        """
        Detects yellow and white lane markers and calculates the error 
        for P-control lane following.
        """
        # Convert image to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Define the ROI (bottom portion of the image)
        height, width = img.shape[:2]
        roi_height = int(height * self.roi_percentage)
        roi = img[height - roi_height:height, 0:width]
        hsv_roi = hsv[height - roi_height:height, 0:width]
        
        # Define the yellow color range (left lane)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([40, 255, 255])
        self.mask_yellow = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
        
        # Define the white color range (right lane)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        self.mask_white = cv2.inRange(hsv_roi, lower_white, upper_white)
        
        # Calculate centroids of the lane markers
        M_yellow = cv2.moments(self.mask_yellow)
        M_white = cv2.moments(self.mask_white)
        
        # Initialize centroids
        cx_yellow, cx_white = 0, width
        
        # Calculate yellow centroid if detected
        if M_yellow["m00"] > 0:
            cx_yellow = int(M_yellow["m10"] / M_yellow["m00"])
        
        # Calculate white centroid if detected
        if M_white["m00"] > 0:
            cx_white = int(M_white["m10"] / M_white["m00"])
        
        # Calculate the center between lane markers
        center = (cx_yellow + cx_white) // 2
        
        # Calculate error: positive when robot needs to turn right, negative for left
        error = center - width // 2
        
        return error

    def send_command(self, v, omega):
        """
        Publishes velocity commands to the Duckiebot.
        
        Args:
            v (float): Linear velocity
            omega (float): Angular velocity (steering)
        """
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        self.car_cmd_pub.publish(car_cmd_msg)

    def manuver_around_bot(self):
        """
        Maneuvers around the detected Duckiebot using a state machine approach.
        """
        # Start the maneuver if not already in progress
        if not self.maneuvering:
            self.maneuvering = True
            self.maneuver_step = 0
            self.maneuver_start_time = rospy.get_time()
            rospy.loginfo("[SAFE NAV] Starting avoidance maneuver")
            
        current_time = rospy.get_time()
        elapsed_time = current_time - self.maneuver_start_time
        
        # State machine for maneuvering
        if self.maneuver_step == 0:
            # Step 0: Stop behind the obstacle
            self.send_command(0, 0)
            rospy.loginfo("[SAFE NAV] Stopping behind the bot")
            if elapsed_time > 1.0:  # Stop for 1 second
                self.maneuver_step = 1
                self.maneuver_start_time = current_time
                
        elif self.maneuver_step == 1:
            # Step 1: Move to the opposite side of where the obstacle is
            # If obstacle is on left, move right; if on right, move left
            steering = 1.0 if self.obstacle_side == "left" else -1.0
            self.send_command(self.maneuver_speed, steering)
            rospy.loginfo(f"[SAFE NAV] Moving to the {('right' if self.obstacle_side == 'left' else 'left')}")
            
            if elapsed_time > 2.0:  # Turn for 2 seconds
                self.maneuver_step = 2
                self.maneuver_start_time = current_time
                
        elif self.maneuver_step == 2:
            # Step 2: Move forward past the obstacle
            self.send_command(self.maneuver_speed, 0)
            rospy.loginfo("[SAFE NAV] Moving forward past the obstacle")
            
            if elapsed_time > 3.0:  # Move forward for 3 seconds
                self.maneuver_step = 3
                self.maneuver_start_time = current_time
                
        elif self.maneuver_step == 3:
            # Step 3: Return to the original lane
            steering = -1.0 if self.obstacle_side == "left" else 1.0
            self.send_command(self.maneuver_speed, steering)
            rospy.loginfo("[SAFE NAV] Returning to original lane")
            
            if elapsed_time > 2.0:  # Turn back for 2 seconds
                self.maneuver_step = 4
                self.maneuver_start_time = current_time
                
        elif self.maneuver_step == 4:
            # Step 4: Resume normal lane following
            rospy.loginfo("[SAFE NAV] Maneuver complete, resuming lane following")
            self.maneuvering = False
            self.obstacle_detected = False
            self.obstacle_pub.publish(Bool(False))  # Signal that the path is now clear

    def lane_following(self, img):
        """
        Implements P-control for lane following.
        """
        error = self.detect_lane_markers(img)
        
        # Calculate steering using P control
        omega = self.lane_p_gain * error
        
        # Limit the steering angle
        omega = max(min(omega, 8.0), -8.0)
        
        # Send command to motors
        self.send_command(self.base_speed, omega)
        
        return error

    def image_callback(self, msg):
        """
        Process incoming camera image to detect obstacles and follow lanes.
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Check for obstacles if not currently maneuvering
        if not self.maneuvering:
            self.obstacle_detected = self.detect_bot(img)
            
            if self.obstacle_detected:
                self.obstacle_pub.publish(Bool(True))
                # Begin maneuver sequence
                self.maneuvering = True
                self.maneuver_start_time = rospy.get_time()
            else:
                # Normal lane following
                error = self.lane_following(img)
                self.last_error = error
        else:
            # Continue with current maneuver
            self.manuver_around_bot()

if __name__ == '__main__':
    # create the node
    node = SafeNavigationNode(node_name='safe_navigation_node')
    rospy.spin()