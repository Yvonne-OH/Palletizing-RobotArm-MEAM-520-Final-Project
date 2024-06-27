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


# BLue grab position
# [-2.10535123  1.01677975  0.50319099 -1.16210212 -2.12033052  3.55645052
# -0.92487665]



# Blue pre-grab
# The joint angle positions are: [-2.24349895  0.69896332  0.45332957 -1.5018697  -2.30542716  3.74539539
# -0.81328175]


# Red Grab
# [ 2.03011955  0.60984668 -0.46281576 -1.84657324  0.70952153  3.14514785
#   0.46966653]



# Red pre-grab
# [ 2.16340264  0.3476119  -0.4625854  -2.30795204  0.71305166  3.53857139
#   0.56017701]


