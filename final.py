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

import numpy as np


def rotate_end_effector(target, axis, angle):
    # 根据旋转轴和角度创建旋转矩阵
    if axis.lower() == 'x':
        rotation_matrix = np.array([
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ])
    elif axis.lower() == 'y':
        rotation_matrix = np.array([
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ])
    elif axis.lower() == 'z':
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")

    rotated_target = np.dot(target, rotation_matrix)
    return rotated_target

def translate_end_effector(target, axis, distance):
    # 创建位移矩阵


    if axis.lower() == 'x':
        target[0, 3] += distance
    elif axis.lower() == 'y':
        target[1, 3] += distance
    elif axis.lower() == 'z':
        target[2, 3] += distance
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")
    return target

import numpy as np

def adjust_pose(Pose):
    if np.abs(Pose[2, 1]) > 0.93:
        Pose[:3, :3] = np.concatenate((Pose[:3, 2].reshape(3, 1), Pose[:3, 0].reshape(3, 1), Pose[:3, 1].reshape(3, 1)), axis=1)
    if np.abs(Pose[2, 0]) > 0.93:
        Pose[:3, :3] = np.concatenate((Pose[:3, 1].reshape(3, 1), Pose[:3, 2].reshape(3, 1), Pose[:3, 0].reshape(3, 1)), axis=1)
    return Pose

def compute_object_pose(Pose, H_ee_camera, T, T_obj_to_end):
    Pose = np.array(Pose)

    if Pose.shape != (4, 4) or H_ee_camera.shape != (4, 4) or T.shape != (4, 4):
        raise ValueError("Input parameter errors")

    # Adjust block pose in camera frame
    Pose = adjust_pose(Pose)

    # Compute block pose in end-effector frame
    pose_end_frame= H_ee_camera @ Pose
    if pose_end_frame[2, 2] < 0:
        pose_end_frame = pose_end_frame @ T_obj_to_end

    """Find a direction of rotation that minimizes the energy of 
    the attitude matrix resulting from rotations in this direction"""
    norm_T = 9999
    Pose_end_frame = np.array(pose_end_frame)
    for pose_i in range(4):
        pose_end_frame = pose_end_frame @ np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        if np.linalg.norm(pose_end_frame - np.eye(4)) < norm_T:
            Pose_end_frame = np.array(pose_end_frame)
            norm_T = np.linalg.norm(pose_end_frame - np.eye(4))
    #block-->world frame
    Trans_block_robo = np.dot(T,Pose_end_frame)
    return Trans_block_robo

def move_to_search_position():
    q_static_search=np.array([ 0.28856799,0.20482551,0.08279217,-1.06908351,-0.01759427,1.27327017,1.14987698])
    arm.safe_move_to_position(q_static_search)
    print(FK.forward(q_static_search))

def check_rms_threshold(q_target, q_current, threshold=0.1):
    # 如果向量长度不同，则无法比较
    if len(q_target) != len(q_current):
        raise ValueError("向量长度不匹配")

    # 计算两个向量的差值
    diff = np.array(q_target) - np.array(q_current)

    # 计算均方根
    rms = np.sqrt(np.mean(diff ** 2))

    # 检查均方根是否小于阈值
    if rms < threshold:
        return True
    else:
        return False




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


    # STUDENT CODE HERE

    seed = start_position

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

    move_to_search_position()

    p, T = FK.forward(arm.get_positions())
    H_ee_camera = detector.get_H_ee_camera()

    Stacked_Layers=0

    # Detect and go through all the static blocks
    for (name, pose) in detector.get_detections():
        print(name, '\n', pose)

        Pose = np.array(pose)
        # Get the block pose in camera frame

        Block_pos_robot_frame = compute_object_pose(Pose, H_ee_camera, T, T_obj_to_end)
        arm.open_gripper()
        Block_pos_robot_frame[2, 3] += 0.10
        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, seed,
                                                                                  method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(q_pseudo)
        Block_pos_robot_frame[2, 3] -= 0.10
        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, seed=q_pseudo,
                                                                                  method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(q_pseudo)
        arm.close_gripper()

        # move_to_search_position()
        Block_target_robot_frame = np.array([
            [1.00000000e+00, -1.86573745e-09, -5.89874375e-09, 5.62000000e-01],
            [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, -1.69000000e-01],
            [-5.89874377e-09, 2.44297209e-09, -1.00000000e+00, 2.50000000e-01],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])


        Block_target_robot_frame[2, 3] += 0.15+0.05*Stacked_Layers
        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed,
                                                                                  method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(q_pseudo)
        Block_target_robot_frame[2, 3] -= 0.15
        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame,
                                                                                  seed=q_pseudo,
                                                                                  method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(q_pseudo)
        arm.open_gripper()
        Block_target_robot_frame[2, 3] += 0.15
        q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame,
                                                                                  seed=q_pseudo,
                                                                                  method='J_pseudo', alpha=.5)
        arm.safe_move_to_position(q_pseudo)
        move_to_search_position()

        Stacked_Layers += 1



    """


    #while (check_goal(q_pseudo, arm.get_positions())):
        #pass
    #arm.close_gripper()
    #print(" ")
    
    
    # Move around...
    
    # END STUDENT CODE"""
