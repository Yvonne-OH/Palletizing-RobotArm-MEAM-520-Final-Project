# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import matplotlib.pyplot as plt
import math


def generate_gripper_coordinates(x, y, angle, distance=0.08, n=8):
    angle_rad = angle
    x_values = np.linspace(x - distance * np.cos(angle_rad), x + distance * np.cos(angle_rad), n)
    y_values = np.linspace(y - distance * np.sin(angle_rad), y + distance * np.sin(angle_rad), n)
    coordinates = list(zip(x_values, y_values))
    return coordinates


def count_intersecting_circles(obstacle_map, gripper, radius1=0.05 * np.sqrt(2) / 2, radius2=0.01):
    count = 0

    for center1 in obstacle_map:
        for center2 in gripper:
            distance = math.sqrt((center1[0] - center2[0]) ** 2 + (center1[1] - center2[1]) ** 2)
            radius_sum = radius1 + radius2

            if distance <= radius_sum:
                count += 1
    print('Collision: ', count)
    return count
    '''
    if count>0:
        return False
    else:
        return True'''

def plot_circles(center_list1, center_list2, radius1=0.05 * np.sqrt(2) / 2, radius2=0.01):
    plt.figure(figsize=(8, 8))

    theta = np.linspace(0, 2 * np.pi, 100)

    for center1 in center_list1:
        x1 = center1[0] + radius1 * np.cos(theta)
        y1 = center1[1] + radius1 * np.sin(theta)
        plt.plot(x1, y1, label=f'Circle 1 ({center1[0]}, {center1[1]})')

    for center2 in center_list2:
        x2 = center2[0] + radius2 * np.cos(theta)
        y2 = center2[1] + radius2 * np.sin(theta)
        plt.plot(x2, y2, label=f'Circle 2 ({center2[0]}, {center2[1]})')

    plt.scatter(*zip(*center_list1), color='red', label='Center 1')
    plt.scatter(*zip(*center_list2), color='blue', label='Center 2')

    intersection_count = count_intersecting_circles(center_list1, center_list2, radius1, radius2)

    if intersection_count > 0:
        plt.title(f'Intersecting Circles: {intersection_count} pairs')
    else:
        plt.title('Non-Intersecting Circles')

    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    # plt.legend()
    plt.grid(True)
    plt.show()

def calculate_collision_numbers(all_block_pose,x_values, y_values, z_angle_values):
    collision_number = []

    for i in range(len(all_block_pose)):
        Gripper_pos_1 = generate_gripper_coordinates(x_values[i], y_values[i], z_angle_values[i])
        Gripper_pos_2 = generate_gripper_coordinates(x_values[i], y_values[i], z_angle_values[i] - np.pi/2)

        obstacle_map = list(zip(x_values, y_values))
        del obstacle_map[i]

        collision_number.append([count_intersecting_circles(obstacle_map, Gripper_pos_1),
                                 count_intersecting_circles(obstacle_map, Gripper_pos_2)])

    return collision_number

if __name__ == "__main__":
    # test x,y,angle
    list1 = [0.09752074935148179, -0.0392296663099628, 0.08312810453619048, -0.022641815384444264]
    list2 = [0.10381267217407808, 0.08177107575861853, -0.032326048285268305, -0.02820315410015206]
    list3 = [1.5143295016180043, -3.0063257544245756, 1.5144602796619497, 1.7464095527857644]
    x_values = np.array(list1)
    y_values = np.array(list2)
    angle = np.array(list3)

    # Test
    center1_list = list(zip(x_values, y_values))
    del center1_list[1]  # delete tarjet from map
    result_coordinates = generate_gripper_coordinates(x_values[1], y_values[1], angle[1])
    count_intersecting_circles(center1_list, result_coordinates)

    plot_circles(center1_list, result_coordinates)

