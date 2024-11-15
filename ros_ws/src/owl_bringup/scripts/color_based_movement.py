#!/usr/bin/env python3

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatusArray
import sys
from moveit_commander.conversions import pose_to_list

class ColorBasedMovement:
    def __init__(self):
        rospy.init_node('color_based_movement', anonymous=True)
        
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Initialize the move group
        self.group_name = "manipulator"  # Replace with your robot's group name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Color ranges in HSV
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'green': ([50, 100, 100], [70, 255, 255]),
            'yellow': ([25, 100, 100], [35, 255, 255])
        }
        
        # Define trajectories for each color
        self.trajectories = {
            'red': self.get_red_trajectory(),
            'blue': self.get_blue_trajectory(),
            'green': self.get_green_trajectory(),
            'yellow': self.get_yellow_trajectory()
        }
        
        # Subscribe to camera feed
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # Flag to track if robot is currently moving
        self.is_moving = False
        
        # Subscribe to move_group status to track motion completion
        self.status_sub = rospy.Subscriber('/move_group/status', GoalStatusArray, self.status_callback)
        
        rospy.loginfo("Color-based movement node initialized")

    def get_red_trajectory(self):
        # Define waypoints for red trajectory
        waypoints = []
        
        # Example waypoints - Replace with your actual coordinates
        wpose = geometry_msgs.msg.Pose()
        
        # Point 1
        wpose.position.x = 0.4
        wpose.position.y = 0.1
        wpose.position.z = 0.4
        wpose.orientation.w = 1.0
        waypoints.append(wpose.deepcopy())
        
        # Point 2
        wpose.position.y = -0.1
        waypoints.append(wpose.deepcopy())
        
        return waypoints

    def get_blue_trajectory(self):
        # Define waypoints for blue trajectory
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        
        # Example waypoints - Replace with your actual coordinates
        wpose.position.x = 0.3
        wpose.position.y = 0.3
        wpose.position.z = 0.3
        wpose.orientation.w = 1.0
        waypoints.append(wpose.deepcopy())
        
        return waypoints

    def get_green_trajectory(self):
        # Define waypoints for green trajectory
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        
        # Example waypoints - Replace with your actual coordinates
        wpose.position.x = 0.2
        wpose.position.y = 0.2
        wpose.position.z = 0.5
        wpose.orientation.w = 1.0
        waypoints.append(wpose.deepcopy())
        
        return waypoints

    def get_yellow_trajectory(self):
        # Define waypoints for yellow trajectory
        waypoints = []
        wpose = geometry_msgs.msg.Pose()
        
        # Example waypoints - Replace with your actual coordinates
        wpose.position.x = 0.3
        wpose.position.y = -0.2
        wpose.position.z = 0.4
        wpose.orientation.w = 1.0
        waypoints.append(wpose.deepcopy())
        
        return waypoints

    def execute_trajectory(self, color):
        """Execute the trajectory for the specified color"""
        if color in self.trajectories and not self.is_moving:
            self.is_moving = True
            waypoints = self.trajectories[color]
            
            # Set max velocity and acceleration scaling factors
            self.move_group.set_max_velocity_scaling_factor(0.5)
            self.move_group.set_max_acceleration_scaling_factor(0.5)
            
            # Plan trajectory
            (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints,   # waypoints to follow
                0.01,        # eef_step
                0.0         # jump_threshold
            )
            
            if fraction >= 0.8:  # If at least 80% of trajectory is valid
                rospy.loginfo(f"Executing trajectory for color: {color}")
                self.move_group.execute(plan, wait=True)
            else:
                rospy.logwarn(f"Could not compute valid path for {color} trajectory")
                self.is_moving = False

    def status_callback(self, data):
        """Monitor movement status"""
        if not data.status_list:
            self.is_moving = False

    def detect_color(self, image):
        """Detect the most prominent color in the image"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        max_area = 0
        detected_color = None
        
        for color_name, (lower, upper) in self.color_ranges.items():
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.inRange(hsv, lower, upper)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Find the largest contour area for this color
            if contours:
                area = max([cv2.contourArea(cnt) for cnt in contours])
                if area > max_area and area > 500:  # Minimum area threshold
                    max_area = area
                    detected_color = color_name
        
        return detected_color

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            detected_color = self.detect_color(cv_image)
            
            if detected_color and not self.is_moving:
                rospy.loginfo(f"Detected color: {detected_color}")
                self.execute_trajectory(detected_color)
            
            # Display the processed image
            cv2.imshow("Color Detection", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        node = ColorBasedMovement()
        node.run()
    except rospy.ROSInterruptException:
        pass