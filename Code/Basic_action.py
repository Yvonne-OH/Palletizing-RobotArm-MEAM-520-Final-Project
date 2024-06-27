import numpy as np
import rospy
import time
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
import Collision_detection

def Gripper_control(arm,command):
    if command=="open":
        while (arm.get_gripper_state().get('position')[0]<0.0375):
            #arm.open_gripper()
            #arm.exec_gripper_cmd(0.05,force=0.1)
            
            arm.exec_gripper_cmd(arm._gripper.MAX_WIDTH * (1 - 1e-2),force=0.1)
            rospy.sleep(1.0)
            print("Opening gripper.... Pos:",arm.get_gripper_state().get('position')[0])
    if command == "close":
        while (arm.get_gripper_state().get('position')[0]>0.03):
            arm.exec_gripper_cmd(0.0,force=0)
            #time.sleep(1.0)
            rospy.sleep(1.0)
            print("closing gripper.... Pos:",arm.get_gripper_state().get('position')[0])
            #arm.close_gripper()

def move_to_static_initial_search_position(arm,team):
    t = time_in_seconds()
    if team=="blue":
        #blue
        q_static_search=np.array([0.28856799,0.20482551,0.08279217,-1.06908351,-0.01759427,1.27327017,1.14987698])
    else:
        #red
        q_static_search=[-0.20603583,0.2071489,- 0.18643621, - 1.06894567, 0.03989059,1.27282757,0.40853973]

    arm.safe_move_to_position(q_static_search)
    print("move_to_static_search_position: ", time_in_seconds() - t)
    print("Search Pos arrived!")

def move_to_static_pre_search_position(arm,team):
    t = time_in_seconds()
    if team=="blue":
        #blue
        q_static_search=np.array([ 0.19967448,0.04253223 , 0.15891895 ,-1.68476423 ,-0.00680944 , 1.72674117,1.14490333])
    else:
        #red
        q_static_search=[-0.17560717,0.04270053,- 0.18358759,- 1.68477457,0.00787935,1.72673995,0.42513846]

    arm.safe_move_to_position(q_static_search)
    print("move_to_static_pre_search_position: ", time_in_seconds() - t)
    print("Pre Search Pos arrived!")

def static_pre_grab(arm,Block_H,IK_pos,seed):
    t = time_in_seconds()
    #arm.open_gripper()
    Gripper_control(arm, "open")
    Block_pos_robot_frame=np.copy(Block_H)
    Block_pos_robot_frame[2, 3] += 0.10
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, seed,
                                                                              method='J_pseudo', alpha=.5)
    print("static_pre_grab_Plan_Time: ", time_in_seconds() - t)
    arm.safe_move_to_position(q_pseudo)
    print("static_pre_grab_Time: ", time_in_seconds() - t)
    print(arm.get_gripper_state())
    return "success"

def static_grab(arm,Block_H,IK_pos,seed):
    t = time_in_seconds()
    Block_pos_robot_frame=np.copy(Block_H)
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, seed=seed,
                                                                              method='J_pseudo', alpha=.5)
    print("static_grab_Plan_Time: ", time_in_seconds() - t)
    arm.safe_move_to_position(q_pseudo)
    #arm.close_gripper()
    Gripper_control(arm, "close")
    print(arm.get_gripper_state())

    Block_pos_robot_frame[2][3]+=0.10
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_pos_robot_frame, seed=seed,
                                                                              method='J_pseudo', alpha=.5)
    arm.safe_move_to_position(q_pseudo)


    print("static_grab_Time: ", time_in_seconds() - t)
    return "success"

def static_place(arm,Target_H,Stacked_Layers,IK_pos,seed):
    t=time_in_seconds()
    Block_target_robot_frame = np.copy(Target_H)
    Block_target_robot_frame[2, 3] += (0.05+0.05 * Stacked_Layers)
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=seed,
                                                                              method='J_pseudo', alpha=.5)
    arm.safe_move_to_position(q_pseudo)
    Block_target_robot_frame[2, 3] -= 0.05
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=arm.get_positions(),
                                                                              method='J_pseudo', alpha=.5)
    #print("pre_place", q_pseudo)
    print("static_place_Plan_Time: ", time_in_seconds() - t)
    arm.safe_move_to_position(q_pseudo)
    print(arm.get_gripper_state())
    #arm.open_gripper()
    Gripper_control(arm, "open")
    print(arm.get_gripper_state())
    print("static_place_Time: ", time_in_seconds() - t)

    return "success"

def static_leave(arm,Target_H,IK_pos,seed):
    t = time_in_seconds()
    Block_target_robot_frame = np.copy(Target_H)
    Block_target_robot_frame[2, 3] += 0.10
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(Block_target_robot_frame, seed=seed,
                                                                              method='J_pseudo', alpha=.5)
    print("static_Leave_Plan_Time: ", time_in_seconds() - t)
    arm.safe_move_to_position(q_pseudo)
    #arm.open_gripper()
    Gripper_control(arm, "open")
    print("static_Leave_Time: ", time_in_seconds() - t)
    return "success"


