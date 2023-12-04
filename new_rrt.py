import numpy as np
import random
from detectCollision import detectCollision
from loadmap import loadmap
from copy import deepcopy
from calculateFK import FK


"""CLASS TREENODE Cited from 
(1) https://stackoverflow.com/questions/58896039/how-do-i-make-treenode-from-list-by-python
(2) https://github.com/motion-planning/rrt-algorithms
"""

def isRobotCollided(map, q):
    """
    :param map:         the map struct
    :param q:           the configuration of the robot
    :return:            returns True if the robot is in collision with the environment, False otherwise
    """
    obstacles = map  # Extracting obstacles from the map structure

    # Checking if there are any obstacles present in the map
    if len(obstacles) > 0:
        fk = FK()  # Initializing Forward Kinematics (FK) object

        # Computing joint positions based on the robot configuration q using FK
        joint_positions, _ = fk.forward(q)

        # Looping through each obstacle in the environment
        for obs_idx in range(len(obstacles)):
            box = obstacles[obs_idx, :].copy()  # Obtaining the current obstacle's dimensions

            # Iterating over each link of the robot to check for collisions with the obstacle
            for i in range(0, 7):
                linePt1 = joint_positions[i, :].reshape(1, 3).copy()  # Start point of a link
                linePt2 = joint_positions[i+1, :].reshape(1, 3).copy()  # End point of a link
                # Checking for collision between the line segment (representing a link) and the obstacle
                if True in detectCollision(linePt1, linePt2, box):
                    return True  # Return True if collision is detected

    return False  # Return False if no collisions are found between the robot and any obstacles in the environment


def isPathCollided(map, q1, q2,start):
    # Function to check if the path between q1 and q2 is collided

    step = 0.01  # Step size for interpolation along the path
    dq = q2 - q1  # Computing the vector between q1 and q2
    n_steps = int(np.linalg.norm(dq) / step)  # Calculating the number of steps needed based on step size

    for i in range(n_steps):
        q = q1 + i * step * dq  # Interpolating points along the path using step size
        if isGripperCollided(map, q,start):  # Checking if the interpolated point is collided
            return True  # Return True if any point on the path is collided
    return False  # Return False if none of the points on the path are collided


def isGripperCollided(all_block_pose,q,start):
    fk = FK()
    __, T_now = fk.forward(start)
    __ , T0e = fk.forward(q)
    T_trans = np.linalg.inv(T_now) @ T0e
    all_block_later = []
    for T in all_block_pose:
        all_block_later.append(T@T_trans)
    x_values, y_values, z_angle_values=Helper_function.extract_pose_values(all_block_later)
    collision_number=Collision_detection.calculate_collision_numbers(all_block_later,x_values, y_values, z_angle_values)
    retuen collision_number

# The RRT algorithm implementation
def rrt(all_block_pose, start, goal, N_samples=4321):
    """
    RRT algorithm without TreeNode structure.
    """
    tree = [{'config': start, 'parent': None}]
    
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])


    for _ in range(N_samples):
        # random tree node 
        q_rand = np.random.uniform(lowerLim, upperLim)

        if isGripperCollided(all_block_pose,q_rand,start):
            continue

        nearest_index = None  # Initialize variable to store the index of the nearest node
        min_dist = float('inf')  # Initialize minimum distance with a very large value
        
        index = 0  # Initialize index variable to keep track of the current node's index in the loop
        
        # Iterate through each node in the tree list
        for node in tree:
            # Calculate the Euclidean distance between the current node's configuration and q_rand
            dist = np.linalg.norm(np.array(node['config']) - q_rand)
        
            # Check if the calculated distance is smaller than the minimum distance found so far
            if dist < min_dist:
                min_dist = dist  # Update the minimum distance
                nearest_index = index  # Update the index of the nearest node
        
            index += 1  # Move to the next index
        
        # Retrieve the nearest node based on the index found
        nearest_node = tree[nearest_index]
        if isPathCollided(all_block_pose, q_rand, nearest_node,start):
            continue
        tree.append({'config': q_rand, 'parent': nearest_index})# Add new tree node

        # Check if the path from q_rand to the goal configuration is collision-free
        if not isPathCollided(all_block_pose, q_rand, goal,start):
            # Create a node for the goal configuration and add it to the tree
            goal_node = {'config': goal, 'parent': len(tree) - 1}
            tree.append(goal_node)
        
            # Trace the path by moving backward through parent nodes
            path = []
            current_node_index = len(tree) - 1  # Start from the newly added goal node
            while current_node_index is not None:
                node = tree[current_node_index]
                path.append(node['config'])  # Store the configuration in the path list
                current_node_index = node['parent']  # Move to the parent node
        
            # Return the path in reverse order as a NumPy array
            return np.array(path[::-1])

    return np.array([])

if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))