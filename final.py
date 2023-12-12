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
from Basic_action import Gripper_control
import Dynamic_Basic_action
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

    #################################### Static Blocks #######################################
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
        if team == 'blue':
            H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(), 'x', 0.00)   # 12.11 Robot2 需要重新调整
            H_ee_camera = Frame_Trans.EE_cam_offset(H_ee_camera, 'y', 0.0)
        else:
            H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(), 'x', 0.005)   # 12.11
            H_ee_camera = Frame_Trans.EE_cam_offset(H_ee_camera, 'y', 0.02)
        # H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(), 'x', 0.035) 
        # H_ee_camera = Frame_Trans.EE_cam_offset(H_ee_camera, 'y', -0.01)


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
    print("Static Task Complete! ", "Time: ", time_in_seconds()-t_start)    
    
    #################################### Dynamic Blocks #######################################
    Place_seed=np.array([-0.12021102-0.3 , 0.20148171 ,-0.17911063 ,-2.02175873 , 0.0447598 ,  2.21961924,0.46256746])
    if team == 'blue':
        Pre_grab_pos = np.array([ 0.17542, -1.06209, -1.74489, -2.01193, -0.31041,  2.16587, -0.69733]) # 12.4 ################## grab_H y-axis = -0.58
        # grab_pos = np.array([0.73588, -1.39156, -1.73435, -1.11631, -0.5597,   1.80013, -0.64694])# 12.4 #######################
        grab_pos = np.array([0.74096, -1.37193, -1.79418, -1.12827, -0.508,    1.83376, -0.67953])# 12.7 ####################### Robot 2 grab
        # Solved by IK using:
        # grab_H = np.array([[ 0.70710678, 0, 0.70710678,  4e-4],
        #                 [0, -1, 0, -0.75],
        #                 [0.70710678, 0, -0.70710678,  0.215],  # 12.7 降低了抓取高度
        #                 [ 0.,  0., 0., 1. ]]
        #                 )
        Dyn_Block_target_robot_frame = np.array([
                [0.70710678, -1.86573745e-09, 0.70710678, 5.62000000e-01],
                [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, -1.69000000e-01],
                [0.70710678, 2.44297209e-09, -0.70710678, 2.38000000e-01],  # 12.8 梅杰： 我暂时降低了放置位置的高度，到时候可能要调整
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ])
        Pre_place_pos = np.array([-0.17131, -0.29179, -0.14215, -1.98463,  0.2873,  2.42964, 0.31471]) # 12.6 Block_target_robot_frame z-axis = 0.6m
    else:
        Pre_grab_pos = np.array([0.48809,  0.51159,  0.96594, -2.00384,  0.42852,  2.13287, -1.2014 ]) # 12.4 ##################
        # grab_pos = np.array([0.95701,  0.99621,  0.74759, -1.13606,  0.17998,  1.87404, -0.96179])
        grab_pos = np.array([0.98493,  0.99437,  0.69333, -1.14293,  0.2234,   1.88858, -0.99483]) # 12.7 Robot 1 grab
        # Solved by IK using:
        # target = np.array([[ -0.70710678, 0, -0.70710678,  0],
        #                     [0, 1, 0, 0.75],
        #                     [0.70710678, 0, -0.70710678,  0.215],     ## 12.7 降低了抓取高度
        #                     [ 0.,  0., 0., 1. ]]
        #                     )   
        Dyn_Block_target_robot_frame = np.array([
                [0, 1, 0, 5.57000000e-01],                      # 12.9 x -0.005m (0.5cm) y -0.007m (0.7cm)
                [0.70710678, 0, 0.70710678, 1.6900000e-01],    # 12.8 梅杰： 我暂时降低了放置位置的高度，到时候可能要调整
                [0.70710678, 0, -0.70710678, 2.38000000e-01],   # 12.7 To 顾恩霖： 由于抓取不在方块正中心，记得改这个放置位置的坐标，不要太高扔下去，x和y可能也有偏移。不行的话dynamic就叠另一堆
                [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
            ]) 
        Pre_place_pos = np.array([0.01909,  0.03751,  0.01723, -1.48918,  0.78321, 1.56588, -0.72771]) # 12.6       
        
    arm.safe_move_to_position(Pre_grab_pos) ### Move to position above the pre-grab!        
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
            # return 0.038 <= width <= 0.051 # Real condition
            return 0.015 <= width <= 0.051 # Real condition # 12.9
        # test = is_block_grabbed(arm)
        while not rospy.is_shutdown(): # IMPORTANT: The loop condition might need to be changed!
            Gripper_control(arm, "close")
            print("loop grip closed")
            # Check if the block is grabbed
            if is_block_grabbed(arm):
                print("Block grabbed successfully!")
                # If grabbed successfully, put the block to the aim stack
                arm.safe_move_to_position(Pre_place_pos)
                Dynamic_Basic_action.dynamic_place(arm,Dyn_Block_target_robot_frame,Stacked_Layers,IK_pos,Place_seed) # Name Change to: Dyn_Block_target_robot_frame
                Dynamic_Basic_action.dynamic_leave(arm, FK.forward(arm.get_positions())[1], IK_pos,arm.get_positions())
                break
            else:
                print("Grab unsuccessful, retrying...")
            # arm.open_gripper()
            Gripper_control(arm, "open")
            rospy.sleep(7.0) # 12.7：增加开爪子时间
            print("loop grip opened!")
        arm.safe_move_to_position(Pre_grab_pos) ### Move to position above the pre-grab!
        Stacked_Layers += 1    
        
    

