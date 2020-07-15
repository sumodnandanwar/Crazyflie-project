#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 19 15:10:34 2020

@author: Fredrik Forsberg
"""

import numpy as np


def get_obj_info(obj_type):
    # obj_type: height, width, shape
    #
    # 1: No bicycle
    
    return {1: (0.192, 0.192, 'circle')}[obj_type]


def image_to_camera(boundry_box_list, obj_type_list, camera_matrix, pitch=None, roll=None):
    return_values = []
    for i in range(0, len(obj_type_list)):
        
        boundry_box = np.asarray([boundry_box_list[i][0:2], [boundry_box_list[i][2], boundry_box_list[i][1]], 
                                  boundry_box_list[i][2:4], [boundry_box_list[i][0], boundry_box_list[i][3]]], 
                                 dtype=np.float64)
        obj_height, obj_width, obj_shape = get_obj_info(obj_type_list[i])
        return_values.append(image_to_camera_per_object(boundry_box, camera_matrix, obj_height, obj_width))
    return return_values


def image_to_camera_per_object(boundry_box, camera_matrix, obj_height, obj_width):
    # The points of the boundry box
    polygon_points = np.ones((3, len(boundry_box)), dtype=np.float64)
    for i in range(0, len(boundry_box)):
        polygon_points[0][i] = boundry_box[i][0]
        polygon_points[1][i] = boundry_box[i][1]
    
    # Cet the inverse camera matrix
    inverse_calibration_matrix = np.linalg.inv(camera_matrix)
    
    # Transform the points
    transformed_points = inverse_calibration_matrix.dot(polygon_points)
    
    # Bounding box dimensions
    # Finding the distance between the pair of "most vertical" connected points
    corner_vectors = [(transformed_points[:, (i+1)%4] - transformed_points[:, i%4]) for i in range(0, 4)]
    verticality = [np.abs(np.array([0, 1, 0]).dot(v/np.linalg.norm(v))) for v in corner_vectors]
    box_height = np.linalg.norm(corner_vectors[verticality.index(max(verticality))])
    
    # Scale according to the height difference
    depth_points = transformed_points * obj_height / box_height
    
    # Center
    center = np.mean(depth_points, axis=1)
    
    # Rotational angle compared to +z
    corner_vectors = [(depth_points[:, (i+1)%4] - depth_points[:, i%4]) for i in range(0, 4)]
    verticality = [np.abs(np.array([0, 1, 0]).dot(v/np.linalg.norm(v))) for v in corner_vectors]
    depth_width = np.linalg.norm(corner_vectors[verticality.index(min(verticality))])
    
    if depth_width/obj_width >= 1:
        rotations = (np.pi, np.pi)
    else:
        # Angle (positive or negative) compared to -z
        diff_angle = np.pi/2. - np.arcsin(depth_width/obj_width)
        # Rotation compared to +z
        rotations = (np.pi - diff_angle, np.pi + diff_angle)
    
    return center, rotations


if __name__ == '__main__':
    calibration_camera_matrix = np.array([[222.07316193,   0.        , 320.],
                                          [  0.        , 220.78075211, 240.],
                                          [  0.        ,   0.        ,   1.]])
    
    obj_height, obj_width = (0.2, 0.2)
    
    boundry_box = np.array([[320-50, 240-50], [320+50, 240-50], [320+50, 240+50], [320-50, 240+50]], dtype=np.float64)
    return_value = image_to_camera_per_object(boundry_box, calibration_camera_matrix, obj_height, obj_width)
    print('\nCenter position:')
    print(return_value[0])
    print('\nPossible rotational angles in degrees (compared to +z, so 180 is perfectly facing the camera):')
    print(np.degrees(return_value[1][0]))
    print(np.degrees(return_value[1][1]))
