#!/usr/bin/env python3

"""
Python example to demonstrate Orangewood SDK APIs with joint-based control and base rotation.
"""

import time
import sys
import signal
import math
from owl_robot_sdk import OwlSimClient

def degrees_to_radians(degrees):
    return [math.radians(angle) for angle in degrees]

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

    # Set SDK APIs to move the robot according to the Swastik joint angles
    def set_sdk_api(self):
        # Define the Swastik joint configurations in degrees
        follower = [
            [2.5821, -0.7815, 0.1563, -2.4266, -0.7288, 0.5174],
            [0, 0, 0, 0, 0, 0],
            [-0.02440, 0.9432, 0.1745, 0.004, 0.0438, -0.0521],
            [0, 0, 0 ,0 ,0 ,0],
            [-2.5918, -1.0217, -0.2074, 2.6183, -0.4956, 2.7581],
            [0, 0, 0 ,0 ,0 ,0],
        ]

        # Execute the movements
        for joint_goals in follower:
            self.move_to_joint(joint_goals)
            time.sleep(0.2)  # Add a delay between movements for visualization

        print("Swastik drawing complete")

        # After completing the task, move the robot to home position
        print("Setting Home")
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