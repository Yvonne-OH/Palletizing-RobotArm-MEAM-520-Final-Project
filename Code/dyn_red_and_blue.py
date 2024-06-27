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
    if team == 'blue':
        Pre_grab_pos = np.array([ 0.17542, -1.06209, -1.74489, -2.01193, -0.31041,  2.16587, -0.69733]) # 12.4 ################## grab_H y-axis = -0.58
        grab_pos = np.array([0.73588, -1.39156, -1.73435, -1.11631, -0.5597,   1.80013, -0.64694])# 12.4 #######################
            # Solved by IK using:
            # grab_H = np.array([[ 0.70710678, 0, 0.70710678,  4e-4],
            #                 [0, -1, 0, -0.75],
            #                 [0.70710678, 0, -0.70710678,  0.23],
            #                 [ 0.,  0., 0., 1. ]]
            #                 )
        ##############################################################################################################################################
        Block_target_robot_frame = np.array([
                [0.70710678, -1.86573745e-09, 0.70710678, 5.62000000e-01],
                [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, -1.69000000e-01],
                [0.70710678, 2.44297209e-09, -0.70710678, 2.50000000e-01],
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ])
        # Pre_place_pos = np.array([0.03951, -0.29319, -0.32459, -2.22497,  0.28642,  2.6866,   0.26751]) # 12.4 Block_target_robot_frame z-axis = 0.5m
        Pre_place_pos = np.array([-0.17131, -0.29179, -0.14215, -1.98463,  0.2873,  2.42964, 0.31471]) # 12.6 Block_target_robot_frame z-axis = 0.6m

    else:
        # Pre_grab_pos = np.array([ 2.16340264,  0.3476119,  -0.4625854,  -2.30795204,  0.71305166,  3.53857139,  0.56017701]) # 12.1 ##################
        # grab_pos = np.array([ 2.03011955,  0.60984668, -0.46281576, -1.84657324,  0.70952153,  3.14514785,  0.46966653])# 12.1 #######################
        ################################################################################################################################################
        Pre_grab_pos = np.array([0.48809,  0.51159,  0.96594, -2.00384,  0.42852,  2.13287, -1.2014 ]) # 12.4 ##################
        grab_pos = np.array([0.95701,  0.99621,  0.74759, -1.13606,  0.17998,  1.87404, -0.96179])
        # Solved by IK using:
        # target = np.array([[ -0.70710678, 0, -0.70710678,  0],
        #                     [0, 1, 0, 0.75],
        #                     [0.70710678, 0, -0.70710678,  0.23],
        #                     [ 0.,  0., 0., 1. ]]
        #                     )   
        ##############################################################################################################################################
        Block_target_robot_frame = np.array([
                [0, 1, 0, 5.62000000e-01],
                [0.70710678, 0, 0.70710678, 1.69000000e-01],
                [0.70710678, 0, -0.70710678, 2.50000000e-01],
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ]) 
        # Pre_place_pos = np.array([0.05229, -0.00035,  0.00407, -1.76799,  0.79962,  1.74955, -0.88582]) # 12.4 
        Pre_place_pos = np.array([0.01909,  0.03751,  0.01723, -1.48918,  0.78321, 1.56588, -0.72771]) # 12.6


    FK=FK()
    IK_pos=lib.IK_position_null.IK()
    # arm.safe_move_to_position(np.array([-0.74949,  0.67183, -0.51995, -1.42384,  0.34769,  2.00397+1, 0.707]))
    # arm.safe_move_to_position(np.array([-0.79092,  0.64844+0.175, -0.57464, -1.65605,  0.41212,  2.18094+1,  0.8429 ])) # bianyuan
    # arm.safe_move_to_position(np.array([-1.11199,  1.33319, -0.53348, -0.6103,   0.20876,  2.7061,   1.11345])) ### Nice Result
    # Dynamic_Basic_action.move_to_dynamic_initial_search_position(arm)
    p, T = FK.forward(arm.get_positions())

    H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(),'x',0.0)
    Stacked_Layers=0

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
            # return width <= 0.07 # Temp
            # return 0.05 <= width <= 0.07 # Simulation
            return 0.038 <= width <= 0.051 # Real condition

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
            rospy.sleep(5.0) # 12.6：增加开爪子时间
            print("loop grip opened!")
        arm.safe_move_to_position(Pre_grab_pos) ### Move to position above the pre-grab!
        Stacked_Layers += 1
