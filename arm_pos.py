"""
Date: 09/11/2023

Purpose: This script creates an ArmController and uses it to command the arm's
joint positions and gripper.

Try changing the target position to see what the arm does!

Student: Jie (Jim) Mei

"""

import sys
import rospy
import numpy as np
from math import pi

from core.interfaces import ArmController

rospy.init_node('demo')

arm = ArmController()
arm.set_arm_speed(0.2)

print("The joint angle positions are:",arm.get_positions())
