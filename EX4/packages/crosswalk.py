#!/usr/bin/env python3

# potentially useful for part 2 of exercise 4

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool
from cv_bridge import CvBridge

class CrossWalkNode(DTROS):

    def __init__(self, node_name):
        super(CrossWalkNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # add your code here

        # call navigation control node
        self.crosswalk_pub = rospy.Publisher(
            f"/{rospy.get_namespace()}crosswalk_detected", Bool, queue_size=1
        )
        self.pedestrian_pub = rospy.Publisher(
            f"/{rospy.get_namespace()}pedestrian_detected", Bool, queue_size=1
        )

        # subscribe to camera feed
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            f"/{rospy.get_namespace()}camera_node/image/compressed", CompressedImage, self.image_callback
        )

        # define other variables as needed
        self.crosswalk_detected = False
        self.pedestrian_detected = False

        rospy.loginfo("CrossWalkNode initialized.")

    def detect_line(self, **kwargs):
        """
        Detect blue crosswalk lines.
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        blue_pixels = cv2.countNonZero(mask)

        if blue_pixels > 500:  # Threshold to detect a crosswalk line
            return True
        return False

    def detect_ducks(self, **kwargs):
        """
        Detect PeDuckstrians using color filtering.
        Uses yellow color filtering to detect small objects in the scene.
        Use small red region (duck mouth) to distictly tell it is a duck and not a road marking
        (Also using shape contour to differentiate)
        """
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        # yellow_pixels = cv2.countNonZero(mask)

        # if yellow_pixels > 200:  # Threshold to detect a PeDuckstrian
        #     return True
        # return False
        # Find yellow contours
        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            # Compute contour area
            area = cv2.contourArea(cnt)

            # Ignore small noise and very large blobs
            if area < 200 or area > 5000:
                continue

            # Fit a minimum enclosing circle
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            circularity = (4 * np.pi * area) / (cv2.arcLength(cnt, True) ** 2 + 1e-5)

            # Ensure the shape is roughly circular (circularity > 0.7)
            if circularity < 0.7:
                continue  # Likely a road marking

            # Looking for a Red Region Inside the Yellow Blob (Duck Mouth) ----
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

            # Create a bounding box for the yellow contour
            x, y, w, h = cv2.boundingRect(cnt)
            red_in_yellow = red_mask[y:y + h, x:x + w]

            # Count red pixels inside the detected yellow blob
            red_pixels = cv2.countNonZero(red_in_yellow)

            if red_pixels > 20:  # Duck mouth threshold
                rospy.loginfo("Detected PeDuckstrian!")
                return True  # A PeDuckstrian is confirmed

        return False  # No valid PeDuckstrians detected

    def image_callback(self, **kwargs):
        """
        Process incoming camera image to detect crosswalk lines and PeDuckstrians.
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Check for blue crosswalk lines
        self.crosswalk_detected = self.detect_line(img)
        self.pedestrian_detected = self.detect_ducks(img)

        # Publish detection results
        self.crosswalk_pub.publish(Bool(self.crosswalk_detected))
        self.pedestrian_pub.publish(Bool(self.pedestrian_detected))

if __name__ == '__main__':
    # create the node
    node = CrossWalkNode(node_name='april_tag_detector')
    rospy.spin()
