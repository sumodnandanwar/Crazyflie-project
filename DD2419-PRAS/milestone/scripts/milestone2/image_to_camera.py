#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 19 15:10:34 2020

Transforms 

@author: Fredrik Forsberg
"""

import numpy as np
from scipy.optimize import least_squares

###


def get_obj_info(obj_type):
    # obj_type: height, width, shape
    #
    # 1: No bicycle
    
    return {1: (0.192, 0.192, 'circle')}[obj_type]
    


def get_rotational_matrix_from_rotational_vector(rot_vector, order=("x", "y", "z")):
    # Code besed on rotation of the +x axis
    if len(rot_vector) != 3:
        raise ValueError('Incorrect argument for rot_vector in get_rotational_matrix_from_rotational_vector: ' +
                         str(rot_vector))

    if order != ("x", "y", "z") and list(set(order)) != ["z", "y", "x"]:
        raise ValueError('Incorrect argument for order in get_rotational_matrix_from_rotational_vector: ' + str(order))

    translation = {"z": "yaw", "y": "pitch", "x": "roll"}
    translated_order = tuple([translation[x] for x in order])

    return get_rotational_matrix_from_yaw_pitch_roll(*(-rot_vector), order=translated_order)


def get_rotational_matrix_from_yaw_pitch_roll(roll, pitch, yaw, order=("roll", "pitch", "yaw")):
    # Code besed on rotation of the +x axis
    # http://planning.cs.uiuc.edu/node102.html
    # Returns the rotational matrix. The middle angle in order must be +-pi/2 (+-90 degrees) while the other two angles
    # can be +-pi (+-180 degrees). All angles in radians.
    r = {"yaw": np.asarray([[np.cos(yaw), -np.sin(yaw), 0],
                            [np.sin(yaw), np.cos(yaw), 0],
                            [0, 0, 1]], dtype=np.float64),
         "pitch": np.asarray([[np.cos(pitch), 0, np.sin(pitch)],
                              [0, 1, 0],
                              [-np.sin(pitch), 0, np.cos(pitch)]], dtype=np.float64),
         "roll": np.asarray([[1, 0, 0],
                             [0, np.cos(roll), -np.sin(roll)],
                             [0, np.sin(roll), np.cos(roll)]], dtype=np.float64)}

    return r[order[2]].dot(r[order[1]]).dot(r[order[0]])


def line_intersection(pt1,pt2,pt3,pt4):
    # By John Ktejik from https://stackoverflow.com/a/59023880
    #least squares method
    def errFunc(estimates):
        s, t = estimates
        x = pt1 + s * (pt2 - pt1) - (pt3 + t * (pt4 - pt3))
        return x

    estimates = [1, 1]

    sols = least_squares(errFunc, estimates)
    s,t = sols.x

    x1 =  pt1[0] + s * (pt2[0] - pt1[0])
    x2 =  pt3[0] + t * (pt4[0] - pt3[0])
    y1 =  pt1[1] + s * (pt2[1] - pt1[1])
    y2 =  pt3[1] + t * (pt4[1] - pt3[1])
    z1 =  pt1[2] + s * (pt2[2] - pt1[2])
    z2 = pt3[2] + t * (pt4[2] - pt3[2])

    x = (x1 + x2) / 2.  #halfway point if they don't match
    y = (y1 + y2) / 2.  # halfway point if they don't match
    z = (z1 + z2) / 2.  # halfway point if they don't match

    return np.array([x,y,z], dtype=np.float64)


def normalize(vector):
    return vector / np.linalg.norm(vector)

###


def image_to_camera(boundry_box_list, obj_type_list, camera_matrix, pitch, roll):
    return_values = []
    for i in range(0, len(obj_type_list)):
        
        boundry_box = np.asarray([boundry_box_list[i][0:2], [boundry_box_list[i][2], boundry_box_list[i][1]], 
                                  boundry_box_list[i][2:4], [boundry_box_list[i][0], boundry_box_list[i][3]]], 
                                 dtype=np.float64)
        obj_height, obj_width, obj_shape = get_obj_info(obj_type_list[i])
        return_values.append(image_to_camera_per_object(boundry_box, camera_matrix, obj_height, obj_width, obj_shape, 
                                                        -pitch, roll))
    return return_values


def image_to_camera_per_object(boundry_box, camera_matrix, obj_height, obj_width, obj_shape, pitch, roll):
    # The points of the boundry box
    polygon_points = np.ones((3, len(boundry_box)), dtype=np.float64)
    for i in range(0, len(boundry_box)):
        polygon_points[0][i] = boundry_box[i][0]
        polygon_points[1][i] = boundry_box[i][1]
    
    # Cet the inverse camera matrix
    inverse_calibration_matrix = np.linalg.inv(camera_matrix)
    
    # Get rotational matrix (the functions are based on rotation around +x, so the input might seem strange for +z)
    rotational_matrix = get_rotational_matrix_from_rotational_vector(np.asarray([-pitch, 0., roll], dtype=np.float64), 
                                                                     order=("x", "y", "z"))
    
    # Transform the points
    transformed_points = rotational_matrix.dot(inverse_calibration_matrix.dot(polygon_points))
    
    # Find the center of the transformed points
    initial_center = np.mean(transformed_points, axis=1).reshape(3, 1)
    
    # Find the horizontal unit vector of the plane, together with -y, perpendicular to our view
    neg_y = np.array([0, -1, 0]).reshape(3, 1)
    h_unit = normalize(np.cross(neg_y.T, initial_center.T).T)  # Numpy cross product requires the three columns
    
    # Find the normal of the plane
    normal = normalize(np.cross(neg_y.T, h_unit.T).T)
    
    for i in range(0, transformed_points.shape[1]):
        
        v = transformed_points[:, i].reshape(3, 1)
    
        # Find the expansion factor of the vector so is's in the plane (using my fancy formula)
        expand_factor = np.float64(np.dot(normal.T, initial_center) / np.dot(normal.T, v))
    
        # Change the vector in transformed_points
        transformed_points[:, i] = (v * expand_factor).reshape(3)
    
    # Bounding box dimensions
    # Finding the height based on "verticality"
    corner_vectors = [(transformed_points[:, (i+1)%4] - transformed_points[:, i%4]) for i in range(0, 4)]
    verticality = [np.abs(np.dot(neg_y.reshape(3), normalize(v))) for v in corner_vectors]
    
    if (verticality[0] + verticality[2]) <= (verticality[1] + verticality[3]):
        vertical_indices = [[0, 1], [3, 2]]
        horizontal_indices = [[1, 2], [0, 3]]
    else:
        vertical_indices = [[1, 2], [0, 3]]
        horizontal_indices = [[0, 1], [3, 2]]
        
    height_list = np.zeros(4)
    for i in range(0, 4):
        opposite_point_indices = vertical_indices[int(i in vertical_indices[0])]
        intersection = line_intersection(transformed_points[:, i], transformed_points[:, i] + neg_y.reshape(3), 
                                         transformed_points[:, opposite_point_indices[0]], 
                                         transformed_points[:, opposite_point_indices[1]])
        height_list[i] = np.abs(np.linalg.norm(transformed_points[:, i] - intersection))
    box_height = np.mean(height_list)
    
    # Scale according to the height difference
    depth_points = transformed_points * obj_height / box_height
    
    # Center
    unrotated_center = np.mean(depth_points, axis=1).reshape(3, 1)
    camera_rotated_center = np.linalg.inv(rotational_matrix).dot(unrotated_center)
    
    # Rotational angle compared to +z
    # Get the width of the boundry box with compensated depth
    depth_h_unit = normalize(np.cross(neg_y.T, unrotated_center.T).T)
    width_list = np.zeros(4)
    for i in range(0, 4):
        opposite_point_indices = horizontal_indices[int(i in horizontal_indices[0])]
        intersection = line_intersection(depth_points[:, i], depth_points[:, i] + depth_h_unit.reshape(3), 
                                         depth_points[:, opposite_point_indices[0]], 
                                         depth_points[:, opposite_point_indices[1]])
        width_list[i] = np.abs(np.linalg.norm(depth_points[:, i] - intersection))
    depth_width = np.mean(width_list)
    
    if obj_shape == 'circle':
        
        if depth_width/obj_width >= 1:
            rotations = (np.pi, np.pi)
        else:
            # Angle (positive or negative) compared to -z
            diff_angle = np.pi/2. - np.arcsin(depth_width/obj_width)
            # Rotation compared to +z
            rotations = (np.pi - diff_angle, -np.pi + diff_angle)
    
    else:
        # TODO Take the rotation of the corners and their inpact on the boundry box into account
        raise(NotImplementedError('Rectangle and triangle have not been implemented yet'))
    
    return camera_rotated_center, rotations


if __name__ == '__main__':
    calibration_camera_matrix = np.array([[222.07316193,   0.        , 320.],
                                          [  0.        , 220.78075211, 240.],
                                          [  0.        ,   0.        ,   1.]])
    
    pitch, roll = (0, 0)  # In radians
    boundry_box_list = np.array([[320-50, 240-50, 320+50, 240+50]], dtype=np.float64)
    obj_type_list = [1]
    
    return_value = image_to_camera(boundry_box_list, obj_type_list, calibration_camera_matrix, pitch, roll)
    
    print(return_value)
    print('\nCenter position:')
    print(return_value[0][0])
    print('\nPossible rotational angles in degrees (compared to +z, so 180 is perfectly facing the camera):')
    print(np.degrees(return_value[0][1][0]))
    print(np.degrees(return_value[0][1][1]))
