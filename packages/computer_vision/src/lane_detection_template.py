#!/usr/bin/env python3

# potentially useful for question - 1.1 - 1.4 and 2.1

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo, Image
import numpy as np

import cv2
from cv_bridge import CvBridge

RED=1
GREEN=2
BLUE=3
YELLOW=4
WHITE=5

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # add your code here
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._camera_info_topic = f"/{self._vehicle_name}/camera_node/camera_info"
        self._distorted_img_topic = f"/{self._vehicle_name}/camera_node/image/compressed"
        self._undistorted_img_topic = f"/{self._vehicle_name}/camera_node/undistorted_image/compressed"
        self._color_detection_topic = f"/{self._vehicle_name}/lane_detection_node/color_detection/compressed"

        # camera calibration parameters (intrinsic matrix and distortion coefficients)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.got_camera_info = False
        self.mapx = None
        self.mapy = None
        self.roi = None
        self.camera_info_sub = rospy.Subscriber(self._camera_info_topic, CameraInfo, self.camera_info_callback)

        # color detection parameters in HSV format
        
        # initialize bridge and subscribe to camera feed
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(self._distorted_img_topic, CompressedImage, self.callback)


        # lane detection publishers
        self.undistorted_pub = rospy.Publisher(self._undistorted_img_topic, CompressedImage, queue_size=1)
        self.color_detection_pub = rospy.Publisher(self._color_detection_topic, CompressedImage, queue_size=1)
        # LED
        
        # ROI vertices
        self.roi = None
        
        # define other variables as needed
        self.rate = rospy.Rate(20)

    def camera_info_callback(self, msg):
        if not self.got_camera_info:
            self.camera_matrix = np.array(msg.K).reshape((3,3))
            self.dist_coeffs = np.array(msg.D)

            rospy.loginfo(f"Camera Intrinsic Matrix: \n{self.camera_matrix}\n")
            rospy.loginfo(f"Distortion Coefficients: \n{self.dist_coeffs}\n")
            
            self.camera_info_sub.unregister()

            #Find the new Optimal Camera matrix
            h,w = 480, 640 # Image shape was found from exercise 2
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs,
                                                                   (w,h), 1, (w,h))
            
            # Find the undistortion mapping matrix to be used for all images
            self.mapx, self.mapy = cv2.initUndistortRectifyMap(self.camera_matrix, self.dist_coeffs, 
                                                               None, new_camera_matrix, (w,h), cv2.CV_32FC1) #cv2.CV_32FC1=5

            self.roi = roi
            self.got_camera_info = True

    def undistort_image(self, image):
        # add your code here
        undistorted_image = cv2.remap(image, self.mapx, self.mapy, cv2.INTER_LINEAR)
        x,y,w,h = self.roi
        return undistorted_image[y:y+h, x:x+w]

    def preprocess_image(self, image):
        # add your code here
        resized_image = cv2.resize(image, (320, 240))
        return cv2.GaussianBlur(resized_image, (5,5), 0)
    
    
    def detect_lane_color(self, image):
        # add your code here
        hsvFrame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        red_lower = np.array([0,95,108], np.uint8)
        red_upper = np.array([8,171,255], np.uint8)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

        green_lower = np.array([44,56,140], np.uint8)
        green_upper = np.array([102,106,188], np.uint8)
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

        blue_lower = np.array([85,110,108], np.uint8)
        blue_upper = np.array([114,238,210], np.uint8)
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)

        kernel = np.ones((5,5), "uint8")

        # For red color 
        red_mask = cv2.dilate(red_mask, kernel) 
        res_red = cv2.bitwise_and(image, image, 
                                mask = red_mask) 
        
        # For green color 
        green_mask = cv2.dilate(green_mask, kernel) 
        res_green = cv2.bitwise_and(image, image, 
                                    mask = green_mask) 
        
        # For blue color 
        blue_mask = cv2.dilate(blue_mask, kernel) 
        res_blue = cv2.bitwise_and(image, image, 
                                mask = blue_mask) 

        # Creating contour to track red color 
        contours, hierarchy = cv2.findContours(red_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y), 
                                        (x + w, y + h), 
                                        (0, 0, 255), 2)
                dimensions_text = f"{w}x{h}"
                cv2.putText(image, dimensions_text, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)     

        # Creating contour to track green color 
        contours, hierarchy = cv2.findContours(green_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
    
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y), 
                                        (x + w, y + h), 
                                        (0, 255, 0), 2) 
                dimensions_text = f"{w}x{h}"
                cv2.putText(image, dimensions_text, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

        # Creating contour to track blue color 
        contours, hierarchy = cv2.findContours(blue_mask, 
                                            cv2.RETR_TREE, 
                                            cv2.CHAIN_APPROX_SIMPLE) 
        for pic, contour in enumerate(contours): 
            area = cv2.contourArea(contour) 
            if(area > 300): 
                x, y, w, h = cv2.boundingRect(contour) 
                image = cv2.rectangle(image, (x, y), 
                                        (x + w, y + h), 
                                        (255, 0, 0), 2) 
                dimensions_text = f"{w}x{h}"
                cv2.putText(image, dimensions_text, (x, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
        return image


    def detect_lane(self, **kwargs):
        # add your code here
        # potentially useful in question 2.1
        pass
    
    
    def callback(self, img_msg):
        # add your code here
        if not self.got_camera_info:
            return
        
        # convert compressed image to CV2
        image = self.bridge.compressed_imgmsg_to_cv2(img_msg)

        # undistort image
        undistorted_image = self.undistort_image(image)

        # preprocess image
        preprocessed_image = self.preprocess_image(undistorted_image)
        
        # detect lanes - 2.1 
        
        # publish lane detection results
        
        # detect lanes and colors - 1.3
        color_detected_image = self.detect_lane_color(preprocessed_image)
        color_detection_msg = self.bridge.cv2_to_compressed_imgmsg(color_detected_image)
        self.color_detection_pub.publish(color_detection_msg)
        
        # publish undistorted image
        undistorted_msg = self.bridge.cv2_to_compressed_imgmsg(undistorted_image)
        self.undistorted_pub.publish(undistorted_msg)
        
        # control LEDs based on detected colors

        # anything else you want to add here
        
        pass

    # add other functions as needed

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    self.cmd_vel_pub = rospy.Publisher(f"/{self._vehicle_name}/cmd_vel", Twist, queue_size=10)
    rospy.spin()