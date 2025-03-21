#!/usr/bin/env python3

# potentially useful for question - 1.5

# import required libraries
import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import Int32
from computer_vision.srv import GetLaneInfo
from time import time

RED = 1  # constant to query red stop line

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        
        # publisher for wheel commands
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self.cmd_topic = f"/{self._vehicle_name}/car_cmd_switch_node/cmd"
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist2DStamped, queue_size=1)

        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params
        self.rate = rospy.Rate(10)
        self.speed = 0.3

        # define other variables as needed
        # AprilTag state
        self.current_tag = -1  # -1 = no detection
        self.stopped = False
        
        # Subscribing to AprilTag ID
        self.tag_sub = rospy.Subscriber(
            f"/{self._vehicle_name}/apriltag_detector_node/tag_id",
            Int32,
            self.tag_callback
        )        

        # Red line detection
        self.lane_info_srv = rospy.ServiceProxy(
            f"/{self._vehicle_name}/lane_detection_node/get_lane_info",
            GetLaneInfo
        )

        # self.rate = rospy.Rate(10)
        rospy.wait_for_service(f"/{self._vehicle_name}/lane_detection_node/get_lane_info")

        rospy.loginfo("NavigationControl initialized. Waiting for AprilTag + red line.")

        self.control_loop()
        
    def publish_velocity(self, **kwargs):
        # add your code here
         = kwargs.get("v", 0.0)
        omega = kwargs.get("omega", 0.0)
        msg = Twist2DStamped()
        msg.header.stamp = rospy.Time.now()
        msg.v = v
        msg.omega = omega
        self.cmd_pub.publish(msg)
        
    def stop(self, **kwargs):
        # add your code here
        duration = kwargs.get("duration", 1.0)
        rospy.loginfo(f"[STOP] Stopping for {duration} seconds.")
        self.publish_velocity(v=0.0, omega=0.0)
        rospy.sleep(duration)
        
    def move_straight(self, **kwargs):
        # add your code here
        distance = kwargs.get("distance", 1.25)
        speed = kwargs.get("speed", self.speed)
        rospy.loginfo(f"[MOVE] Driving forward {distance}m at {speed}m/s")

        duration = distance / speed
        t_start = time()

        while time() - t_start < duration:
            self.publish_velocity(v=speed, omega=0.0)
            self.rate.sleep()

        self.publish_velocity(v=0.0, omega=0.0)
        rospy.loginfo("[MOVE] Done moving forward.")
        
    def turn_right(self, **kwargs):
        # add your code here
        pass
        
    def turn_left(self, **kwargs):
        # add your code here
        pass

    # add other functions as needed
    def control_loop(self):
        """
        It runs the main loop for red line detection + stopping logic
        """
        while not rospy.is_shutdown() and not self.stopped:
            try:
                if self.current_tag == -1:
                    rospy.loginfo_throttle(5, "Waiting for AprilTag detection...")
                    self.rate.sleep()
                    continue

                response = self.lane_info_srv(RED)

                if response.detected and response.distance <= 0.25:
                    rospy.loginfo(f"[RED LINE] Detected at {response.distance:.2f}m")

                    # determine stop duration
                    tag_stop_times = {
                        0: 3.0,  # Stop sign
                        1: 2.0,  # T-intersection
                        2: 1.0   # UofA
                    }
                    stop_time = tag_stop_times.get(self.current_tag, 0.5)

                    self.stop(duration=stop_time)
                    self.move_straight(distance=1.25)
                    self.stopped = True
                    rospy.loginfo("[DONE] AprilTag stop behavior complete.")
            except rospy.ServiceException as e:
                rospy.logerr(f"Red line service error: {e}")

            self.rate.sleep()

if __name__ == '__main__':
    node = NavigationControl(node_name='navigation_control_node')
    rospy.spin()