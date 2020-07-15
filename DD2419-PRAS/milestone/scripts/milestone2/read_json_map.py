#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import numpy as np
import cv2

###


def read_json(path):
    with open(path) as f:
        json_dict = json.load(f)
        
    return json_dict


def create_grid(json_dict, grid_res=100):
    # grid_res is the number of rows/columns per meter => 10 means that each cell represents 1 dm x 1 dm
    offset = json_dict['airspace']['min'][:2]
    print(offset)
    max_point = json_dict['airspace']['max'][:2]
    
    x_len = max_point[0] - offset[0]
    y_len = max_point[1] - offset[1]

    grid = np.zeros((int(x_len*grid_res), int(y_len*grid_res)), dtype=np.byte)

    return grid, np.asarray(offset, dtype=np.float64)


def draw_walls(json_dict, grid, offset, grid_res=100):
    # line_res is how many points per meter there are to be between the start and the stop points of a wall
    line_res = 10 * grid_res
    for  d in json_dict['walls']:
        start = np.array(d['plane']['start'][:2]) - offset
        stop = np.array(d['plane']['stop'][:2]) - offset
        print(start)
        print(stop)

        n_points = int(np.linalg.norm(start - stop) * line_res)
        print(n_points)
        for point in np.linspace(start, stop, n_points):
            floored_point = np.floor(point * (grid_res-1)).astype(dtype=np.int) # The floor of the scalar x is the largest integer i, such that i <= x. 
            #print(floored_point)
            grid[floored_point[0]][floored_point[1]] = 1
            
    return grid


def grid_index_to_world_coord(index, offset, grid_res):
    index = np.asarray(index, dtype=np.float64)
    offset = np.asarray(offset, dtype=np.float64)
    
    return (index / grid_res) + offset


def world_coord_to_grid_index(coord, offset, grid_res):
    coord = np.asarray(coord, dtype=np.float64)
    offset = np.asarray(offset, dtype=np.float64)
    
    return np.floor((coord - offset) * grid_res)
    

###
    

if __name__ == '__main__':
    json_file = '~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json'
    json_dict = read_json(json_file)
    grid, offset = create_grid(json_dict, grid_res=100)
    grid = draw_walls(json_dict, grid, offset, grid_res=100)
    
    # Plot the grid as an image just to make sure it works
    img = np.zeros(tuple(list(grid.shape) + [3]))
    img[:,:,0] = grid * 255
    img[:,:,1] = grid * 255
    img[:,:,2] = grid * 255
    
    cv2.imshow("2D map", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    