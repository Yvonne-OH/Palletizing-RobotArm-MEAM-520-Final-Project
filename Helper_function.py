import numpy as np

def rotate_end_effector(target, axis, angle):
    # The function rotate_end_effector generates a rotation matrix
    # based on the specified axis ('x', 'y', or 'z') and angle,
    # and then applies this rotation to the target matrix, returning the rotated result.
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
    # The function translate_end_effector generates a translation matrix based
    # on the specified axis ('x', 'y', or 'z') and distance,
    # modifying the target matrix accordingly.
    if axis.lower() == 'x':
        target[0, 3] += distance
    elif axis.lower() == 'y':
        target[1, 3] += distance
    elif axis.lower() == 'z':
        target[2, 3] += distance
    else:
        raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")
    return target


def check_rms_threshold(q_target, q_current, threshold=0.1):
    # The function check_rms_threshold compares two vectors q_target and q_current
    # by calculating their root mean square (RMS) difference.
    # If the RMS difference is below a specified threshold (default is 0.1),
    # the function returns True; otherwise, it returns False.

    if len(q_target) != len(q_current):
        raise ValueError("向量长度不匹配")

    diff = np.array(q_target) - np.array(q_current)
    rms = np.sqrt(np.mean(diff ** 2))

    if rms < threshold:
        return True
    else:
        return False


def extract_pose_values(all_block_pose):
    x_values = []
    y_values = []
    z_angle_values = []

    for pose in all_block_pose:
        x_values.append(pose[0, 3])
        y_values.append(pose[1, 3])
        z_angle_values.append(np.arctan2(pose[1, 0], pose[0, 0]))

    return x_values, y_values, z_angle_values

def quickly_cal_H(IK_pos,H_start,x,y,z,rx,ry,rz,seed):

    H_start = translate_end_effector(H_start, 'x', x)
    H_start = translate_end_effector(H_start, 'y', y)
    H_start = translate_end_effector(H_start, 'z', z)
    H_start = rotate_end_effector(H_start, 'x', rx)
    H_start = rotate_end_effector(H_start, 'y', ry)
    H_start = rotate_end_effector(H_start, 'z', rz)
    q_pseudo, rollout_pseudo, success_pseudo, message_pseudo = IK_pos.inverse(H_start, seed=seed,
                                                                              method='J_pseudo', alpha=.5)

    print("Quickly_Determine_q",q_pseudo)
    return q_pseudo
