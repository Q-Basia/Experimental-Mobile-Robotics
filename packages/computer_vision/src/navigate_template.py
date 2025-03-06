#!/usr/bin/env python3

# potentially useful for question - 1.5

# import required libraries

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        # add your code here
        
        # publisher for wheel commands
        # NOTE: you can directly publish to wheel chassis using the car_cmd_switch_node topic in this assignment (check documentation)
        # you can also use your exercise 2 code
        
        # robot params

        # define other variables as needed
        
    def publish_velocity(self, **kwargs):
        # add your code here
        pass
        
    def stop(self, **kwargs):
        rospy.loginfo(f"Stopping for {duration} seconds.")

        # Publish LED color (Red for stop)
        self.set_led_color("red")

        # Stop movement
        velocity_msg = Twist()
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        self.cmd_vel_pub.publish(velocity_msg)

        # Wait for the specified duration
        rospy.sleep(duration)
        pass
        
    def move_straight(self, **kwargs):
        rospy.loginfo(f"Moving straight for {distance} meters at speed {speed}.")
    
        # Publish LED color (White for moving forward)
        self.set_led_color("white")

        # Predefined speed and distance
        speed = 0.2
        distance = 1.0  # Move forward 1 meter

        # Calculate duration based on speed (constant velocity assumed)
        duration = distance / speed
        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_msg = Twist()
            velocity_msg.linear.x = speed  # Move forward
            velocity_msg.angular.z = 0  # No turning
            self.cmd_vel_pub.publish(velocity_msg)
            self.rate.sleep()

        # Stop after moving
        self.stop()
        pass
        
    def turn_right(self, **kwargs):
        rospy.loginfo("Turning right 90 degrees.")

        # Publish LED color (Yellow for right turn)
        self.set_led_color("yellow")

        # Predefined speed and duration
        angular_speed = -0.2  # Negative for right turn
        duration = 2.0  # Estimated time for 90° turn

        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_msg = Twist()
            velocity_msg.linear.x = 0  # No forward movement
            velocity_msg.angular.z = -speed  # Rotate right
            self.cmd_vel_pub.publish(velocity_msg)
            self.rate.sleep()

        # Stop after turning
        self.stop()
        pass
        
    def turn_left(self, **kwargs):
        rospy.loginfo("Turning left 90 degrees.")

        # Publish LED color (Cyan for left turn)
        self.set_led_color("cyan")

        # Predefined speed and duration
        angular_speed = -0.2  # Negative for right turn
        duration = 2.0  # Estimated time for 90° turn

        start_time = rospy.Time.now().to_sec()

        while rospy.Time.now().to_sec() - start_time < duration:
            velocity_msg = Twist()
            velocity_msg.linear.x = 0  # No forward movement
            velocity_msg.angular.z = speed  # Rotate left
            self.cmd_vel_pub.publish(velocity_msg)
            self.rate.sleep()

        # Stop after turning
        self.stop()
        pass

    # add other functions as needed

if __name__ == '__main__':
    node = NavigationControl(node_name='navigation_control_node')
    rospy.spin()