#!/usr/bin/env python3

# potentially useful for question - 1.5

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from lane_detection_template import LaneDetectionNode
import math

VELOCITY = 0.3
OMEGA = 0.0

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        self.lane_detection_node = LaneDetectionNode(node_name="Lane_Color_Detection")

        # publisher for wheel commands
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._twist_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self._vel_publisher = rospy.Publisher(self._twist_topic, Twist2DStamped, queue_size=1)
        # robot params
        self.v = VELOCITY
        self.omega = OMEGA

        # define other variables as needed
        self.rate = rospy.Rate(10)
        
    def publish_velocity(self):
        message = Twist2DStamped(v=self.v, omega=self.omega)
        self._vel_publisher.publish(message)
        
    def stop(self):
        rospy.loginfo(f"Stopping.")
        # Publish LED color (Red for stop)
        self.lane_detection_node.set_led_color([[1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],])
        self.set_led_color("red")
        self.publish_velocity(0,0)

    def wait(self, duration):
        rospy.loginfo(f"Stopping for {duration} seconds.")

        # Publish LED color (Red for stop)
        self.lane_detection_node.set_led_color([[1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],
                                [1, 0, 0, 1],])

        # Stop movement
        self.publish_velocity(0,0)

        # Wait for the specified duration
        rospy.sleep(duration)
        
    def move_straight(self, distance, speed=0.3):
        rospy.loginfo(f"Moving straight for {distance} m")
    
        # Publish LED color (White for moving forward)
        self.lane_detection_node.set_led_color([[1, 1, 1, 1],
                                [1, 1, 1, 1],
                                [1, 1, 1, 1],
                                [1, 1, 1, 1],
                                [1, 1, 1, 1],])

        # Calculate duration based on speed (constant velocity assumed)
        duration = distance / speed
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration:
            self.v = speed
            self.omega = 0
            self.publish_velocity()
            self.rate.sleep()

        # Stop after moving
        self.stop()
        
    def turn_right(self, angle, radius):
        rospy.loginfo("Turning right 90 degrees.")

        # Publish LED color (Yellow (front right and back right) for right turn)
        self.lane_detection_node.set_led_color([[0, 0, 0, 0],
                                [1, 1, 0, 1],
                                [0, 0, 0, 0],
                                [1, 1, 0, 1],
                                [0, 0, 0, 0],])

        self.turn(angle, radius, omega=-math.pi/4)
        
    def turn_left(self, angle, radius):
        rospy.loginfo("Turning left 90 degrees.")

        # Publish LED color (Yellow (front right and back right) for right turn)
        self.lane_detection_node.set_led_color([[1, 1, 0, 1],
                                [0, 0, 0, 0],
                                [1, 1, 0, 1],
                                [0, 0, 0, 0],
                                [0, 0, 0, 0],])

        self.turn(angle, radius, omega=-math.pi/4)
        

    # add other functions as needed
    def turn(self, angle, radius, omega=math.pi/4):

        # linear speed and duration
        duration = angle/omega
        speed = radius*omega

        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration:
            self.v = speed
            self.omega = omega
            self.publish_velocity()
            self.rate.sleep()

        # Stop after turning
        self.stop()

if __name__ == '__main__':
    node = NavigationControl(node_name='navigation_control_node')
    rospy.spin()