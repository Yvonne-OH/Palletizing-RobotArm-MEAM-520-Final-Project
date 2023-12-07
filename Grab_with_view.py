import numpy as np
import rospy
import time
import Frame_Trans
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
import Collision_detection

def Grab_with_view(arm, Target_H, Stacked_Layers, IK_pos, seed, H_ee_camera):
    t = time_in_seconds()
    T_obj_to_end = np.array([[1, 0, 0, 0],
                             [0, -1, 0, 0],
                             [0, 0, -1, 0],
                             [0, 0, 0, 1]])
    Collision_detection_index = [0,0]
    Block_target_robot_frame = np.copy(Target_H)
    # Block_target_robot_frame[2, 3] += (0.05+0.05 * Stacked_Layers)
    Block_target_robot_frame[2, 3] += (0.2+0.05 * Stacked_Layers)
    Block_target_robot_frame[0, 3] -= 0.1
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=seed,
                                                                              method='J_pseudo', alpha=.5)
    arm.safe_move_to_position(q_pseudo)
    Block_target_robot_frame[0, 3] += 0.1
    Block_target_robot_frame[2, 3] -= 0.225
    # In case of inaccuracy, only tune the x and y position, and check z
    # we know that the block orientation is certain after last placement
    # z position: see if the last block falls off
    # basic values
    if Stacked_Layers >= 1:
        FK = FK()
        p, T_now = FK.forward(arm.get_positions())
        dete = ObjectDetector()
        ideal_z = Block_target_robot_frame[2, 3]
        all_block_pose = [pose for name, pose in dete.get_detections()]
        # get the highest block's x, y and z
        high_x = Block_target_robot_frame[0, 3]
        high_y = Block_target_robot_frame[1, 3]
        high_z = Block_target_robot_frame[2, 3]
        # if not detected, do as expected
        print("Number of stack detected", len(all_block_pose), '\n')
        if len(all_block_pose):
            high_z = 0
            for pose in all_block_pose:
                print(pose)
                Tran = Frame_Trans.compute_object_pose(
                    pose, H_ee_camera, T_now, T_obj_to_end, Collision_detection_index)
                print("Transmat", Tran, '\n')
                if high_z < Tran[2, 3]:
                    high_x = Tran[0, 3]
                    high_y = Tran[1, 3]
                    high_z = Tran[2, 3]
            Block_target_robot_frame[0, 3] = high_x
            Block_target_robot_frame[1, 3] = high_y
        # if last block fall off, reduce z
        if high_z < (ideal_z - 0.04):
            print("Last Block Fall!", '\n')
            ideal_z -= 0.05
        Block_target_robot_frame[2, 3] = ideal_z
        print("Place Z: ", Block_target_robot_frame[2, 3], '\n')
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=arm.get_positions(),
                                                                              method='J_pseudo', alpha=.5)
    # print("pre_place", q_pseudo)
    print("static_place_Plan_Time: ", time_in_seconds() - t)

    arm.safe_move_to_position(q_pseudo)
    print(arm.get_gripper_state())
    # arm.open_gripper()
    Gripper_control(arm, "open")
    print(arm.get_gripper_state())
    print("static_place_Time: ", time_in_seconds() - t)

    return "success"
