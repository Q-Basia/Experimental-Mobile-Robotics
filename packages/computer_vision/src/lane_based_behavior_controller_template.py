#!/usr/bin/env python3

# potentially useful for question - 1.6

# import required libraries
import rospy
import os
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class BehaviorController(DTROS):
    def __init__(self, node_name):
        super(BehaviorController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # call navigation control node
        self.navigation_control_node = NavigationControl(node_name="navigate_control_node")

        # define parameters
        
        # Color ranges in HSV

        # LED stuff
        
        # subscribe to camera feed
        self.camera_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self.camera_sub = rospy.Subscriber(self.camera_topic, CompressedImage, self.callback)
        
        # define other variables as needed
        self.bridge = CvBridge()

        rospy.loginfo("BehaviorController Node Initialized.")
        
    def set_led_pattern(self, **kwargs):

        color_map = {
            "red": (1, 0, 0),
            "green": (0, 1, 0),
            "blue": (0, 0, 1),
            "yellow": (1, 1, 0),
            "white": (1, 1, 1),
            "off": (0, 0, 0)
        }

        rgb = color_map.get(color.lower(), (0, 0, 0))  # Default to "off" if invalid
        r, g, b = rgb

        led_pattern = LEDPattern()
        for _ in range(5):  # 5 LEDs on the Duckiebot
            rgba = ColorRGBA()
            rgba.r, rgba.g, rgba.b, rgba.a = r, g, b, 1.0
            led_pattern.rgb_vals.append(rgba)

        self.led_pub.publish(led_pattern)
        rospy.loginfo(f"LEDs set to {color.upper()} ({r}, {g}, {b}).")

        
    def detect_line(self, **kwargs):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define HSV ranges for red, green, and blue
        color_ranges = {
            "red": ([0, 95, 108], [8, 171, 255]),
            "green": ([44, 56, 140], [102, 106, 188]),
            "blue": ([85, 110, 108], [114, 238, 210])
        }

        detected_color = "unknown"
        max_area = 0

        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > max_area:
                    max_area = area
                    detected_color = color

        return detected_color
    
    def execute_blue_line_behavior(self, **kwargs):
        rospy.loginfo("Executing blue line behavior.")
        self.lane_detection.stop()
        self.lane_detection_template.color("yellow")  # Signal right
        rospy.sleep(1)
        self.lane_detection.turn_right()
        pass
        
    def execute_red_line_behavior(self, **kwargs):
        rospy.loginfo("Executing red line behavior.")
        self.lane_detection.stop()
        self.lane_detection.move_straight()
        pass
        
    def execute_yellow_line_behavior(self, **kwargs):
        rospy.loginfo("Executing green line behavior.")
        self.lane_detection.stop()
        self.lane_detection.set_led_color("yellow")  # Signal left
        rospy.sleep(1)
        self.lane_detection.turn_left()
        pass
        
    def callback(self, **kwargs):
        image = self.bridge.compressed_imgmsg_to_cv2(img_msg)
        detected_color = self.lane_detection.get_lane_color(image)  # Reusing function

        if detected_color == "blue":
            self.execute_blue_line_behavior()
        elif detected_color == "red":
            self.execute_red_line_behavior()
        elif detected_color == "green":
            self.execute_green_line_behavior()
        else:
            rospy.loginfo("No relevant color detected.")
        pass

    # add other functions as needed

if __name__ == '__main__':
    node = BehaviorController(node_name='behavior_controller_node')
    rospy.spin()