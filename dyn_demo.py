import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# self-dev Lib
from lib.IK_velocity_null import IK_velocity_null
from lib.calculateFK import FK
from lib.calcAngDiff import calcAngDiff
import lib.IK_position_null
import Helper_function
import Frame_Trans
from Basic_action import Gripper_control
import Dynamic_Basic_action

import numpy as np


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!
    
    # seed = start_position
    # dynamic_pre_grab_seed = np.array([-1.18062,  0.73451, -0.55257, -1.33372,  0.39019,  1.96088,  0.591 ]) 
    Place_seed=np.array([-0.12021102-0.3 , 0.20148171 ,-0.17911063 ,-2.02175873 , 0.0447598 ,  2.21961924,0.46256746])
    # NB_seed = np.array([0,0,0,-pi/2,0,pi/2,pi/4])

    T_cam_to_end = np.array([[0, 1, 0, 0],
                            [-1, 0, 0, 0.05],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    T_end_to_cam = np.linalg.inv(T_cam_to_end)
    T_obj_to_end = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])

    # Pre_grab_pos = np.array([-1.12638,  1.36176-0.25, -0.51741, -0.58107, 0.18119,  2.7103,   1.13345])
    
    ####################################################################################### VERY IMPORTANT! ######################################
    Pre_grab_pos = np.array([-2.24349895,  0.69896332,  0.45332957, -1.5018697,  -2.30542716,  3.74539539, -0.81328175]) # 12.1 ##################
    grab_pos = np.array([-2.10535123,  1.01677975,  0.50319099, -1.16210212, -2.12033052,  3.55645052, -0.92487665])# 12.1 #######################
    ##############################################################################################################################################
    Pre_place_pos = np.array([-0.01779206-0.3, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353, 0.75344866])
    

    FK=FK()
    IK_pos=lib.IK_position_null.IK()
    # arm.safe_move_to_position(np.array([-0.74949,  0.67183, -0.51995, -1.42384,  0.34769,  2.00397+1, 0.707]))
    # arm.safe_move_to_position(np.array([-0.79092,  0.64844+0.175, -0.57464, -1.65605,  0.41212,  2.18094+1,  0.8429 ])) # bianyuan
    # arm.safe_move_to_position(np.array([-1.11199,  1.33319, -0.53348, -0.6103,   0.20876,  2.7061,   1.11345])) ### Nice Result 
    # Dynamic_Basic_action.move_to_dynamic_initial_search_position(arm)
    p, T = FK.forward(arm.get_positions())

    H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(),'x',0.0)

    Stacked_Layers=0
    Block_target_robot_frame = np.array([
            [1.00000000e+00, -1.86573745e-09, -5.89874375e-09, 5.62000000e-01],
            [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, -1.69000000e-01],
            [-5.89874377e-09, 2.44297209e-09, -1.00000000e+00, 2.50000000e-01],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])

    arm.open_gripper()
    arm.safe_move_to_position(Pre_grab_pos) ### Move to position above the pre-grab!
    t = time_in_seconds()
   
    while not rospy.is_shutdown(): # ############ if static tasks is completed NEEDS TO BE CHANGED  IMPORTANT!!!!! ##################
        arm.safe_move_to_position(grab_pos)
        # Judge Function determining whether the block is grabbed successfully.
        def is_block_grabbed(arm):
            gripper_state = arm.get_gripper_state()
            positions = gripper_state['position']
            width = abs(positions[1] + positions[0]) # detect the width between the jaws
            print("The grip width is: ",width)
            return width <= 0.07 # Temp
            # return 0.05 <= width <= 0.07 # Simulation
            # return 0.048 <= width <= 0.055 # Real condition

        # test = is_block_grabbed(arm)
        while not rospy.is_shutdown(): # IMPORTANT: The loop condition might need to be changed!
            Gripper_control(arm, "close")
            print("loop grip closed")
            # Check if the block is grabbed
            if is_block_grabbed(arm):
                print("Block grabbed successfully!")
                # If grabbed successfully, put the block to the aim stack
                arm.safe_move_to_position(Pre_place_pos)
                Dynamic_Basic_action.dynamic_place(arm,Block_target_robot_frame,Stacked_Layers,IK_pos,Place_seed)
                Dynamic_Basic_action.dynamic_leave(arm, FK.forward(arm.get_positions())[1], IK_pos,arm.get_positions())
                break
            else:
                print("Grab unsuccessful, retrying...")
            # arm.open_gripper()
            Gripper_control(arm, "open")
            print("loop grip opened!")
        arm.safe_move_to_position(Pre_grab_pos) ### Move to position above the pre-grab!
        Stacked_Layers += 1
    

