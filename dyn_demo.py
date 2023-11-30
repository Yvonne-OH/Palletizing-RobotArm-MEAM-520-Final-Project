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
import Basic_action
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
    
    seed = start_position
    dynamic_pre_grab_seed = np.array([-1.18062,  0.73451, -0.55257, -1.33372,  0.39019,  1.96088,  0.591 ]) # Dyn Grab Pos
    # NB_seed = np.array([-pi/2,0,0,-pi/2,0,pi/2,pi/4])


    T_cam_to_end = np.array([[0, 1, 0, 0],
                            [-1, 0, 0, 0.05],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
    T_end_to_cam = np.linalg.inv(T_cam_to_end)
    T_obj_to_end = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, 0],
                            [0, 0, 0, 1]])

    FK=FK()
    IK_pos=lib.IK_position_null.IK()

    # arm.safe_move_to_position(np.array([-0.74949,  0.67183, -0.51995, -1.42384,  0.34769,  2.00397+1, 0.707]))
    # arm.safe_move_to_position(np.array([-0.79092,  0.64844+0.175, -0.57464, -1.65605,  0.41212,  2.18094+1,  0.8429 ])) # bianyuan
    arm.safe_move_to_position(np.array([-1.11199,  1.33319, -0.53348, -0.6103,   0.20876,  2.7061,   1.11345])) ### Nice Result Up-to-date DO NOT DELETE!
    # Dynamic_Basic_action.move_to_dynamic_initial_search_position(arm)
    p, T = FK.forward(arm.get_positions())

    H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(),'x',0.0)

    Stacked_Layers=0
    
    ############################
    # Detect the Dynamic Blocks 
    ############################

    # for (name, pose) in detector.get_detections():
    #     print(name, '\n', pose)
    #     Pose = np.array(pose)
    #     # Get the block pose in camera frame
    #     Block_pos_robot_frame = Frame_Trans.compute_object_pose(Pose, H_ee_camera, T, T_obj_to_end)

    #     # Ignore THIS
    #     # print(Block_pos_robot_frame)
    #     # q_test, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, dynamic_pre_grab_seed,
    #                                                                             # method='J_pseudo', alpha=.5)    
    #     # arm.safe_move_to_position(q_test)

    #     Dynamic_Basic_action.dynamic_pre_grab(arm,Block_pos_robot_frame, IK_pos, dynamic_pre_grab_seed)
    #     # Basic_action.static_grab(arm,Block_pos_robot_frame, IK_pos, pre_grab_seed)
    


