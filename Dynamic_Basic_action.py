import numpy as np
import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds


# def move_to_dynamic_initial_search_position(arm): # Search Position for dynamic blocks
#     t = time_in_seconds()
#     q_dynamic_search=np.array([-1.27169, 0.58862, -0.5554,  -0.84653,  0.3033,   1.37183,  0.66958])
#     # q_dynamic_search=np.array([-1.16812,  0.70977, -0.58861, -1.2796,   0.3893,  1.87717,  0.59601]) # lower search altitude
#     arm.safe_move_to_position(q_dynamic_search)
#     print("move_to_dynamic_search_position: ", time_in_seconds() - t)
#     print("Dynamic Search Pos arrived!")

#######################################################################################################

# def move_to_static_pre_search_position(arm):
#     t = time_in_seconds()
#     q_static_search=np.array([ 0.20013175,0.04376772,0.15780585,-1.8106012,-0.00716238,1.8538347, 1.1451915 ])
#     arm.safe_move_to_position(q_static_search)
#     print("move_to_static_pre_search_position: ", time_in_seconds() - t)
#     print("Pre Search Pos arrived!")

# def dynamic_pre_grab(arm,Block_H,IK_pos,seed):
#     t = time_in_seconds()
#     arm.open_gripper()
#     Block_pos_robot_frame=np.copy(Block_H)
#     # Block_pos_robot_frame[0, 3] -= 0.05
#     q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, seed,
#                                                                                 method='J_pseudo', alpha=.5)
#     print("dynamic_pre_grab_Plan_Time: ", time_in_seconds() - t)
#     arm.safe_move_to_position(q_pseudo)
#     print("dynamic_pre_grab_Time: ", time_in_seconds() - t)
#     print("q_pseudo: ",q_pseudo)
#     return "success"

def dynamic_place(arm,Target_H,Stacked_Layers,IK_pos,seed):
    t=time_in_seconds()
    Block_target_robot_frame = np.copy(Target_H)
    Block_target_robot_frame[2, 3] += (0.00+0.05 * Stacked_Layers)
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=seed,
                                                                                method='J_pseudo', alpha=.5)
    print("dynamic_place_Plan_Time: ", time_in_seconds() - t)
    arm.safe_move_to_position(q_pseudo)
    arm.open_gripper()
    print("dynamic_place_Time: ", time_in_seconds() - t)

    return "success"

def dynamic_leave(arm,Target_H,IK_pos,seed):
    t = time_in_seconds()
    Block_target_robot_frame = np.copy(Target_H)
    Block_target_robot_frame[2, 3] += 0.10
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=seed,
                                                                                method='J_pseudo', alpha=.5)
    print("dynamic_Leave_Plan_Time: ", time_in_seconds() - t)
    arm.safe_move_to_position(q_pseudo)
    arm.open_gripper()
    print("dynamic_Leave_Time: ", time_in_seconds() - t)
    return "success"

