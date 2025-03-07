#!/usr/bin/env python3

# potentially useful for question - 1.1 - 1.4 and 2.1

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo, Image, ColorRGBA
import numpy as np
from duckietown_msgs.msg import LEDPattern

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
        self.red_lower = np.array([0,95,108], np.uint8)
        self.red_upper = np.array([8,171,255], np.uint8)
        self.green_lower = np.array([44,56,140], np.uint8)
        self.green_upper = np.array([102,106,188], np.uint8)
        self.blue_lower = np.array([85,110,108], np.uint8)
        self.blue_upper = np.array([114,238,210], np.uint8)

        # w,h of each lane
        self.detected_lanes = {
        'red': {'detected': False, 'contour': None, 'dimensions': None, 'bbox': None},
        'green': {'detected': False, 'contour': None, 'dimensions': None, 'bbox': None},
        'blue': {'detected': False, 'contour': None, 'dimensions': None, 'bbox': None}
        }
        self.red_lane = (0,0)
        self.green_lane = (0,0)
        self.blue_lane = (0,0)

        # initialize bridge and subscribe to camera feed
        self.bridge = CvBridge()
        self.camera_sub = rospy.Subscriber(self._distorted_img_topic, CompressedImage, self.callback)


        # lane detection publishers
        self.undistorted_pub = rospy.Publisher(self._undistorted_img_topic, CompressedImage, queue_size=1)
        self.color_detection_pub = rospy.Publisher(self._color_detection_topic, CompressedImage, queue_size=1)

        # LED
        self.LEDspattern = LEDPattern()
        self.light_color_list = [
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                 ]
        self.leds = {
            "front_left": [1,0,0,1], 
            "front_right": [1,0,0,1],
            "back_left": [1,0,0,1],
            "back_right": [1,0,0,1],
            "center": [1,0,0,1],
            }
        

        self.pub_leds = rospy.Publisher(f"/{self._vehicle_name}/led_emitter_node/led_pattern", LEDPattern, queue_size=10)
        self.count = 1
        
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

        red_mask = cv2.inRange(hsvFrame, self.red_lower, self.red_upper)

        green_mask = cv2.inRange(hsvFrame, self.green_lower, self.green_upper)

        blue_mask = cv2.inRange(hsvFrame, self.blue_lower, self.blue_upper)

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
                self.detected_lanes['red']['detected'] = True
                self.detected_lanes['red']['contour'] = contour
                self.detected_lanes['red']['dimensions'] = (w, h)
                self.detected_lanes['red']['bbox'] = (x, y, w, h)
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
                self.detected_lanes['green']['detected'] = True
                self.detected_lanes['green']['contour'] = contour
                self.detected_lanes['green']['dimensions'] = (w, h)
                self.detected_lanes['green']['bbox'] = (x, y, w, h)
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
                self.detected_lanes['blue']['detected'] = True
                self.detected_lanes['blue']['contour'] = contour
                self.detected_lanes['blue']['dimensions'] = (w, h)
                self.detected_lanes['blue']['bbox'] = (x, y, w, h)
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

    def publish_LED_pattern(self):
        # Publish the LED pattern to the led_emitter_node
        self.LEDspattern.rgb_vals = []
        for i in range(5):
            rgba = ColorRGBA()
            rgba.r = self.light_color_list[i][0]
            rgba.g = self.light_color_list[i][1]
            rgba.b = self.light_color_list[i][2]
            rgba.a = self.light_color_list[i][3]

            self.LEDspattern.rgb_vals.append(rgba)
        self.pub_leds.publish(self.LEDspattern)

    def set_led_color(self, colors):
        # Set the color of the LEDs

        # colors should be a list of length 5 with 
        # each element being a list of length 4
        for i in range(len(self.light_color_list)):
            if len(colors[i]==3):
                self.light_color_list[i] = colors[i] + [1]
            else:
                self.light_color_list[i] = colors[i]
        
        self.publish_LED_pattern()
    
    
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

        # if self.detected_lanes['red']['dimensions'][0]*self.detected_lanes['red']['dimensions'][1] > 500:
        #     # set led colors to red if red lane detected at a large size
        #     self.set_led_color([[1, 0, 0, 1],
        #                         [1, 0, 0, 1],
        #                         [1, 0, 0, 1],
        #                         [1, 0, 0, 1],
        #                         [1, 0, 0, 1],])
            
        # elif self.detected_lanes['green']['dimensions'][0]*self.detected_lanes['green']['dimensions'][1] > 500:
        #     # set led colors to green if green lane detected at a large size
        #     self.set_led_color([[0, 1, 0, 1],
        #                         [0, 1, 0, 1],
        #                         [0, 1, 0, 1],
        #                         [0, 1, 0, 1],
        #                         [0, 1, 0, 1],])
            
        # elif self.detected_lanes['blue']['dimensions'][0]*self.detected_lanes['blue']['dimensions'][1] > 500:
        #     # set led colors to blue if blue lane detected at a large size
        #     self.set_led_color([[0, 0, 1, 1],
        #                         [0, 0, 1, 1],
        #                         [0, 0, 1, 1],
        #                         [0, 0, 1, 1],
        #                         [0, 0, 1, 1],])

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()