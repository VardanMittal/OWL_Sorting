#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class InteractiveColorDetector:
    def __init__(self):
        rospy.init_node('interactive_color_detector', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Color ranges in HSV format
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'blue': ([100, 100, 100], [130, 255, 255]),
            'green': ([50, 100, 100], [70, 255, 255]),
            'yellow': ([25, 100, 100], [35, 255, 255])
        }
        
        # Current color to detect (default to None)
        self.current_color = None
        
        # Subscribe to camera feed
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        
        # Publisher for processed images
        self.image_pub = rospy.Publisher('detected_color_image', Image, queue_size=10)
        
        rospy.loginfo("Interactive Color Detector initialized")
        rospy.loginfo("Available colors: " + ", ".join(self.color_ranges.keys()))
        rospy.loginfo("Type 'select color_name' to detect a specific color")
        
        # Start listening for user input in a separate thread
        self.start_user_input()

    def start_user_input(self):
        """Handle user input in a separate thread"""
        import threading
        input_thread = threading.Thread(target=self.user_input_loop)
        input_thread.daemon = True
        input_thread.start()

    def user_input_loop(self):
        """Continuously listen for user input"""
        while not rospy.is_shutdown():
            try:
                user_input = input().strip().lower()
                if user_input.startswith('select '):
                    color = user_input[7:]  # Remove 'select ' from the input
                    if color in self.color_ranges:
                        self.current_color = color
                        rospy.loginfo(f"Now detecting {color} objects")
                    else:
                        rospy.logwarn(f"Unknown color: {color}")
                        rospy.loginfo("Available colors: " + ", ".join(self.color_ranges.keys()))
            except Exception as e:
                rospy.logerr(f"Error processing input: {e}")

    def detect_color(self, image, color_name):
        """Detect specified color in the image and return bounding boxes"""
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Get color range
        lower, upper = self.color_ranges[color_name]
        lower = np.array(lower)
        upper = np.array(upper)
        
        # Create mask
        mask = cv2.inRange(hsv, lower, upper)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find bounding boxes for significant contours
        boxes = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter small noise
                x, y, w, h = cv2.boundingRect(contour)
                boxes.append((x, y, w, h))
        
        return boxes

    def image_callback(self, data):
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # If a color is selected, detect and draw boxes
            if self.current_color:
                boxes = self.detect_color(cv_image, self.current_color)
                
                # Draw boxes and labels
                for (x, y, w, h) in boxes:
                    # Draw rectangle
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    
                    # Add label
                    label = f"{self.current_color}"
                    cv2.putText(cv_image, label, (x, y - 10),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
                # Display detection count
                if boxes:
                    count_text = f"Found {len(boxes)} {self.current_color} object(s)"
                    rospy.loginfo_throttle(1, count_text)  # Log every second
                
            # Add UI instructions
            cv2.putText(cv_image, "Press 'q' to quit", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Display the image
            cv2.imshow("Color Detection", cv_image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                rospy.signal_shutdown("User requested quit")
            
            # Publish the processed image
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                rospy.logerr(f"Error publishing image: {e}")

        except CvBridgeError as e:
            rospy.logerr(f"Error converting image: {e}")
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
        detector = InteractiveColorDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass