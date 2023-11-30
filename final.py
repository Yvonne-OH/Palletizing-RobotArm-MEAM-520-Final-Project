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

    t_start=time_in_seconds()


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

    Place_seed=np.array([-0.12021102 , 0.20148171 ,-0.17911063 ,-2.02175873 , 0.0447598 ,  2.21961924,0.46256746])

    FK=FK()
    IK_pos=lib.IK_position_null.IK()

    Basic_action.move_to_static_initial_search_position(arm)
    p, T = FK.forward(arm.get_positions())

    H_ee_camera = Frame_Trans.EE_cam_offset(detector.get_H_ee_camera(),'x',0.0)

    Stacked_Layers=0

    Block_num=len(detector.get_detections()[0])
    for i in range(Block_num):
        remain_num=0
        if remain_num<=0:
            break
        pass

    # Detect and go through all the static blocks
    for (name, pose) in detector.get_detections():
        print(name, '\n', pose)

        Pose = np.array(pose)
        # Get the block pose in camera frame

        Block_pos_robot_frame = Frame_Trans.compute_object_pose(Pose, H_ee_camera, T, T_obj_to_end)

        Basic_action.static_pre_grab(arm,Block_pos_robot_frame, IK_pos, arm.get_positions())
        Basic_action.static_grab(arm,Block_pos_robot_frame, IK_pos, arm.get_positions())

        Block_target_robot_frame = np.array([
            [1.00000000e+00, -1.86573745e-09, -5.89874375e-09, 5.62000000e-01],
            [-1.86573734e-09, -1.00000000e+00, -2.44297205e-09, -1.69000000e-01],
            [-5.89874377e-09, 2.44297209e-09, -1.00000000e+00, 2.50000000e-01],
            [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
        ])

        Basic_action.static_place(arm,Block_target_robot_frame,Stacked_Layers,IK_pos,Place_seed)
        Basic_action.static_leave(arm, FK.forward(arm.get_positions())[1], IK_pos,arm.get_positions())


        Basic_action.move_to_static_pre_search_position(arm)

        Stacked_Layers += 1

    print("Complete! ", "Time: ", time_in_seconds()-t_start)

