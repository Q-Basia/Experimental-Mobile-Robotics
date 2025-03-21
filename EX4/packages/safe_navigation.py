#!/usr/bin/env python3

# potentially useful for part 2 of exercise 4

# import required libraries
import rospy
from duckietown.dtros import DTROS, NodeType

class SafeNavigationNode(DTROS):

    def __init__(self, node_name):
        super(SafeNavigationNode, self).__init__(node_name=node_name, node_type=NodeType.LOCALIZATION)

        # add your code here

        # call navigation control node
        self.obstacle_pub = rospy.Publisher(
            f"/{rospy.get_namespace()}obstacle_detected", Bool, queue_size=1
        )

        # subscribe to camera feed
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            f"/{rospy.get_namespace()}camera_node/image/compressed", CompressedImage, self.image_callback
        )

        # define other variables as needed
        self.obstacle_detected = False

        rospy.loginfo("SafeNavigationNode initialized.")

    def detect_bot(self, **kwargs):
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

            # Ignore small blue objects (reflections, random noise)
            if area < 1000:
                continue

            # If a large blue object is found, assume it's a Duckiebot
            rospy.loginfo("Detected a Duckiebot in the way!")
            return True  # Obstacle detected

        return False  # No Duckiebot detected

    def manuver_around_bot(self, **kwargs):
        """
        Maneuvers around the detected Duckiebot.
        """
        rospy.loginfo("[SAFE NAV] Stopping behind the bot for 3 seconds.")
        rospy.sleep(3)

        rospy.loginfo("[SAFE NAV] Maneuvering around...")
        # Implement turning logic (simple left-right avoidance)
        self.obstacle_pub.publish(Bool(False))  # Signal that the path is now clear

    def image_callback(self, **kwargs):
        """
        Process incoming camera image to detect obstacles (Duckiebots).
        """
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        self.obstacle_detected = self.detect_bot(img)

        if self.obstacle_detected:
            self.obstacle_pub.publish(Bool(True))
            self.manuver_around_bot()
        else:
            self.obstacle_pub.publish(Bool(False))

if __name__ == '__main__':
    # create the node
    node = SafeNavigationNode(node_name='april_tag_detector')
    rospy.spin()
