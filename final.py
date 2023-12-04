import sys
import numpy as np
from copy import deepcopy
from math import pi
import time

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

# self-dev Lib
from lib.calculateFK import FK
import lib.IK_position_null
import Helper_function
import Frame_Trans
import Basic_action
import Collision_detection

import numpy as np
import Decision
import matplotlib.pyplot as plt


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

    t_start=time_in_seconds()


    # STUDENT CODE HERE
    T_cam_to_end = np.array([[0, 1, 0, 0],
                             [-1, 0, 0, 0.05],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
    T_end_to_cam = np.linalg.inv(T_cam_to_end)
    T_obj_to_end = np.array([[1, 0, 0, 0],
                             [0, -1, 0, 0],
                             [0, 0, -1, 0],
                             [0, 0, 0, 1]])

    if team == 'blue':
        Block_target_robot_frame = np.array([
            [1.00000000e+00, -1.86573745e-09, -5.89874375e-09, 5.62000000e-01],
            [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, -1.69000000e-01],
            [-5.89874377e-09, 2.44297209e-09, -1.00000000e+00, 2.50000000e-01],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])
    else:
        Block_target_robot_frame = np.array([
            [1.00000000e+00, -1.86573745e-09, -5.89874375e-09, 5.62000000e-01],
            [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, 1.69000000e-01],
            [-5.89874377e-09, 2.44297209e-09, -1.00000000e+00, 2.50000000e-01],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])

    seed = start_position
    Place_seed=np.array([-0.12021102 , 0.20148171 ,-0.17911063 ,-2.02175873 , 0.0447598 ,  2.21961924,0.46256746])

    FK=FK()
    IK_pos=lib.IK_position_null.IK()

    Basic_action.move_to_static_initial_search_position(arm,team)
    p, T = FK.forward(arm.get_positions())

    '''
    print(T)
    T[2][3]-=0.15
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(T, seed,
                                                                              method='J_pseudo', alpha=.5)
    print("pre_search",q_pseudo)
    arm.safe_move_to_position(q_pseudo)'''

    Block_num=len(detector.get_detections())
    print(Block_num)
    Stacked_Layers=0
    Remaining_blocks=Block_num

    for iteration in range(Block_num):
        print("iteration",iteration)
        all_block_pose=[pose for name, pose in detector.get_detections()]
        print("all_block_pose",all_block_pose)
        if len(all_block_pose)>1:
            #print("all_block_pose",all_block_pose)
            x_values, y_values, z_angle_values=Helper_function.extract_pose_values(all_block_pose)
            collision_number=Collision_detection.calculate_collision_numbers(all_block_pose,x_values, y_values, z_angle_values)
            print(collision_number)
            sorted_collision_number, sorted_all_block_pose=Decision.sort_collision_data(collision_number, all_block_pose)
            print("sorted_collision_number", sorted_collision_number)

            Pose = np.array(sorted_all_block_pose[0])
            Pose[2][3]= 4.41717247e-01
            Collision_detection_index=sorted_collision_number[0]
        else:
            Pose = all_block_pose[0]
            print("Pose last:",Pose)
            print("-----------------------")
            Pose[2][3] = 4.41717247e-01
            Collision_detection_index = [0,0]

        #adjust Camera Offset
        H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(), 'x', 0.0)

        print("all_block_pose", Pose)
        print(Collision_detection_index)
        # Get the block pose in camera frame
        Block_pos_robot_frame = Frame_Trans.compute_object_pose(Pose, H_ee_camera, T, T_obj_to_end,Collision_detection_index)

        Basic_action.static_pre_grab(arm,Block_pos_robot_frame, IK_pos, arm.get_positions())
        Basic_action.static_grab(arm,Block_pos_robot_frame, IK_pos, arm.get_positions())

        Basic_action.static_place(arm,Block_target_robot_frame,Stacked_Layers,IK_pos,Place_seed)
        Basic_action.static_leave(arm, FK.forward(arm.get_positions())[1], IK_pos,arm.get_positions())

        Stacked_Layers += 1
        Remaining_blocks -=1

        print('Remaining_blocks',Remaining_blocks)

        if Remaining_blocks>0:
            Basic_action.move_to_static_pre_search_position(arm,team)

    print("Complete! ", "Time: ", time_in_seconds()-t_start)

