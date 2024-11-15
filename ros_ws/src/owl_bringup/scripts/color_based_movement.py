#!/usr/bin/env python3

"""
Python example to demonstrate Orangewood SDK APIs with joint-based control and base rotation.
This version accepts color input (green, blue, red) and maps it to predefined joint angles.
"""

import time
import sys
import signal
import math
from owl_robot_sdk import OwlSimClient

class TestOwlSimClient:
    def __init__(self, group_name, gripper_group):
        self.group_name = group_name

        signal.signal(signal.SIGINT, self.signal_handler)

        print("Application is running. Press Ctrl+C to exit.")

        # Initialize the OwlSimClient for your robot
        self.client = OwlSimClient(self.group_name, gripper_group, 5, True)

        self.get_sdk_api()
        self.set_sdk_api()

    def signal_handler(self, sig, frame):
        print("Interrupt received, closing application.")
        sys.exit(0)

    # Testing Get APIs of SDK
    def get_sdk_api(self):
        status = self.client.is_running()
        print("Robot Running Status:= ", status)

        version = self.client.get_version()
        print("Robot Motion Group:= ", version)

        joint_val = self.client.get_joint()
        print("Robot Joint position:= ", joint_val)

    # Set SDK APIs to move the robot according to the color input
    def set_sdk_api(self):
        # Define color-to-joint-angle mappings (in degrees)
        color_joint_mappings = {
            "base": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "green": [2.5821, -0.7815, 0.1563, -2.4266, -0.7288, 0.5174],
            "blue":  [-2.5918, -1.0217, -0.2074, 2.6183, -0.4956, 2.7581],
            "red": [-0.02440, 0.9432, 0.1745, 0.004, 0.0438, -0.0521]
        }

        while True:
            # Prompt the user to enter a color (green, blue, red)
            print("\nEnter a color (green, blue, or red): ")
            color_input = input("Color: ").strip().lower()

            # Check if the input color is valid
            if color_input in color_joint_mappings:
                # Get the corresponding joint angles for the entered color
                joint_goals = color_joint_mappings[color_input]
                print(f"Received color: {color_input}")
                print(f"Corresponding joint angles: {joint_goals}")

                # Move the robot to the specified joint angles
                
                self.move_to_joint(joint_goals)
                self.move_to_joint(color_joint_mappings["base"])
                time.sleep(1)  # Add delay between moves for visualization

            else:
                print(f"Invalid color input. Please enter one of: green, blue, red.")

            # Ask the user if they want to enter another color
            continue_input = input("Do you want to enter another color? (y/n): ").strip().lower()
            if continue_input != 'y':
                break

        # After completing the task, move the robot to home position
        print("Setting Home position...")
        self.client.set_home()
        time.sleep(5)

    def move_to_joint(self, joint_goals):
        """
        Moves the robot to the specified joint angles.
        """
        status = self.client.move_to_joint(joint_goals)
        print(f"Moving to joint positions: {joint_goals} - Status: {status}")
        time.sleep(0.2)

if __name__ == "__main__":
    # Replace "arm" and "gripper" with the actual group names of your robot
    testowl = TestOwlSimClient("arm", "gripper")
