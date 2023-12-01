import numpy as np
import Collision_detection

def EE_cam_offset(H,axis,distance):
    # adjust EE_cam offset
    if axis.lower() == 'x':
        H[0, 3] += distance
    elif axis.lower() == 'y':
        H[1, 3] += distance
    elif axis.lower() == 'z':
        H[2, 3] += distance
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")
    return H

def adjust_pose(Pose):
    if np.abs(Pose[2, 1]) > 0.93:
        Pose[:3, :3] = np.concatenate((Pose[:3, 2].reshape(3, 1), Pose[:3, 0].reshape(3, 1), Pose[:3, 1].reshape(3, 1)), axis=1)
    if np.abs(Pose[2, 0]) > 0.93:
        Pose[:3, :3] = np.concatenate((Pose[:3, 1].reshape(3, 1), Pose[:3, 2].reshape(3, 1), Pose[:3, 0].reshape(3, 1)), axis=1)
    return Pose

def compute_object_pose(Pose, H_ee_camera, T, T_obj_to_end,Collision_detection):
    Pose = np.array(Pose)

    if Pose.shape != (4, 4) or H_ee_camera.shape != (4, 4) or T.shape != (4, 4):
        raise ValueError("Input parameter errors")

    # Adjust block pose in camera frame
    Pose = adjust_pose(Pose)

    # Compute block pose in end-effector frame
    pose_end_frame= H_ee_camera @ Pose
    if pose_end_frame[2, 2] < 0:
        pose_end_frame = pose_end_frame @ T_obj_to_end

    if (Collision_detection[0]==0)and(Collision_detection[1]==0):
        """Find a direction of rotation that minimizes the energy of 
        the attitude matrix resulting from rotations in this direction"""
        norm_T = 9999
        Pose_end_frame = np.array(pose_end_frame)
        for pose_i in range(4):
            pose_end_frame = pose_end_frame @ np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            if np.linalg.norm(pose_end_frame - np.eye(4)) < norm_T:
                Pose_end_frame = np.array(pose_end_frame)
                norm_T = np.linalg.norm(pose_end_frame - np.eye(4))

    if (Collision_detection[1]==0)and(Collision_detection[0]!=0):
        """Find a direction of rotation that minimizes the energy of 
        the attitude matrix resulting from rotations in this direction"""
        print("case 2")
        norm_T = 9999
        Pose_end_frame = np.array(pose_end_frame)
        for pose_i in range(2):
            pose_end_frame = pose_end_frame @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            if np.linalg.norm(pose_end_frame - np.eye(4)) < norm_T:
                Pose_end_frame = np.array(pose_end_frame)
                norm_T = np.linalg.norm(pose_end_frame - np.eye(4))

    if (Collision_detection[1]!=0)and(Collision_detection[0]==0):
        """Find a direction of rotation that minimizes the energy of 
        the attitude matrix resulting from rotations in this direction"""
        print("case 3")
        norm_T = 9999
        pose_end_frame=pose_end_frame@ np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        Pose_end_frame = np.array(pose_end_frame)

        for pose_i in range(2):
            pose_end_frame = pose_end_frame @ np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            if np.linalg.norm(pose_end_frame - np.eye(4)) < norm_T:
                Pose_end_frame = np.array(pose_end_frame)
                norm_T = np.linalg.norm(pose_end_frame - np.eye(4))
                print("norm",norm_T)

    else:
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
