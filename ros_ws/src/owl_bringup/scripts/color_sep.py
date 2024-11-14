#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detection_node', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create a publisher for detected colors
        self.color_pub = rospy.Publisher('detected_color', String, queue_size=10)
        
        # Subscribe to the Gazebo camera topic
        # Note: Replace '/camera/rgb/image_raw' with your actual camera topic
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # Define color ranges in HSV
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'green': ([50, 100, 100], [70, 255, 255]),
            'yellow': ([25, 100, 100], [35, 255, 255])
        }
        
        rospy.loginfo("Color detection node initialized")

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Convert to HSV color space
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            detected_colors = []
            
            # Check for each color
            for color_name, (lower, upper) in self.color_ranges.items():
                lower = np.array(lower)
                upper = np.array(upper)
                
                # Create mask for current color
                mask = cv2.inRange(hsv_image, lower, upper)
                
                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # If contours are found and they're large enough (to avoid noise)
                if contours and max([cv2.contourArea(cnt) for cnt in contours]) > 500:
                    detected_colors.append(color_name)
                    
                    # Draw contours for visualization
                    cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)
            
            # Publish detected colors
            if detected_colors:
                color_msg = String()
                color_msg.data = ', '.join(detected_colors)
                self.color_pub.publish(color_msg)
                rospy.loginfo(f"Detected colors: {color_msg.data}")
            
            # Display the image (optional)
            cv2.imshow("Color Detection", cv_image)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Shutting down color detection node")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        color_detector = ColorDetectionNode()
        color_detector.run()
    except rospy.ROSInterruptException:
        pass