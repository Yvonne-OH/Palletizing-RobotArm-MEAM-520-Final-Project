

def sort_collision_data(collision_number, all_block_pose):
    if len(collision_number) <= 1 or len(all_block_pose) <= 1:
        return collision_number, all_block_pose

    sorted_indices = sorted(range(len(collision_number)), key=lambda i: (collision_number[i][0], collision_number[i][1]))
    sorted_collision_number = sorted(collision_number, key=lambda x: (x[0], x[1]))
    sorted_all_block_pose = [all_block_pose[i] for i in sorted_indices]

    return sorted_collision_number, sorted_all_block_pose