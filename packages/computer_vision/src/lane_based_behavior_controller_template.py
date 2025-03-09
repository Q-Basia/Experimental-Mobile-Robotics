#!/usr/bin/env python3

# potentially useful for question - 1.6

# import required libraries
import rospy
import os
import cv2
import numpy as np
import argparse
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import LEDPattern
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from navigate_template import NavigationControl
from lane_detection_template import LaneDetectionNode

class BehaviorController(DTROS):
    def __init__(self, node_name):
        super(BehaviorController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here

        self._vehicle_name = os.environ['VEHICLE_NAME']

        # call navigation control node which includes led control
        self.navigation_control_node = NavigationControl(node_name="navigate_control_node")
        # lane detection node used to detect lanes

        #Instead of calling node, make lane detection node publish self.detect_lanes to a custom topic
        #Subscribe to it here
        self.lane_detection_node = LaneDetectionNode(node_name="lane_detect_node")

        # define parameters

        # The homography parameters were exported from the duckiebot dashboard after performing extrinsic calibration
        homography_list = [
        -4.99621433668091e-05, 0.0012704090688819693, 0.2428235605203261,
        -0.001999628080487182, -5.849807527639727e-05, 0.6400119336043912,
        0.0003409556379103712, 0.0174415825291776, -3.2316507961510252
        ]
        self.homography_matrix = np.array(homography_list).reshape((3, 3))

        rospy.loginfo("BehaviorController Node Initialized.")

        
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
        if self.lane_detection_node.detected_lanes['red'] and (self.lane_detection_node.detected_lanes['red']['contour'] is not None):
            contour = self.lane_detection_node.detected_lanes['red']['contour']
            distance = self.get_distance_to_lane(contour, self.homography_matrix)

            
        self.lane_detection.stop()
        self.lane_detection.move_straight()
        pass
        
    def execute_green_line_behavior(self, **kwargs):
        rospy.loginfo("Executing green line behavior.")
        self.lane_detection.stop()
        self.lane_detection.set_led_color("yellow")  # Signal left
        rospy.sleep(1)
        self.lane_detection.turn_left()
        pass

def parse_args():
        parser = argparse.ArgumentParser(description='Lane detection and behavior execution')
        parser.add_argument('--lane', '-l', type=str, choices=['red', 'green', 'blue'], 
                            required=True, help='Lane color to execute behavior for')
        return parser.parse_args()

if __name__ == '__main__':
    node = BehaviorController(node_name='behavior_controller_node')
    args = parse_args()
    if args.lane == "red":
        node.execute_red_line_behavior()
    elif args.lane == "green":
        node.execute_green_line_behavior()
    else:
        node.execute_blue_line_behavior()
        

    rospy.spin()