#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/sumslinux/dd2419_ws/src/localization/scripts")
import json
import numpy as np
import matplotlib.pyplot as plt
from utils import png_to_ogm
import cv2

class OccupancyGridMap:
    def __init__(self, data_array, cell_size, occupancy_threshold=0.8):
        """
        Creates a grid map
        :param data_array: a 2D array with a value of occupancy per cell (values from 0 - 1)
        :param cell_size: cell size in meters
        :param occupancy_threshold: A threshold to determine whether a cell is occupied or free.
        A cell is considered occupied if its value >= occupancy_threshold, free otherwise.
        """

        self.data = data_array
        self.dim_cells = data_array.shape
        self.dim_meters = (self.dim_cells[0] * cell_size, self.dim_cells[1] * cell_size)
        self.cell_size = cell_size
        self.occupancy_threshold = occupancy_threshold
        # 2D array to mark visited nodes (in the beginning, no node has been visited)
        self.visited = np.zeros(self.dim_cells, dtype=np.float32)

    def mark_visited_idx(self, point_idx):
        """
        Mark a point as visited.
        :param point_idx: a point (x, y) in data array
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.visited[y_index][x_index] = 1.0

    def mark_visited(self, point):
        """
        Mark a point as visited.
        :param point: a 2D point (x, y) in meters
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.mark_visited_idx((x_index, y_index))

    def is_visited_idx(self, point_idx):
        """
        Check whether the given point is visited.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is visited, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        if self.visited[y_index][x_index] == 1.0:
            return True
        else:
            return False

    def is_visited(self, point):
        """
        Check whether the given point is visited.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is visited, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_visited_idx((x_index, y_index))

    def get_data_idx(self, point_idx):
        """
        Get the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :return: the occupancy value of the given point
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        return self.data[y_index][x_index]

    def get_data(self, point):
        """
        Get the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :return: the occupancy value of the given point
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.get_data_idx((x_index, y_index))

    def set_data_idx(self, point_idx, new_value):
        """
        Set the occupancy value of the given point.
        :param point_idx: a point (x, y) in data array
        :param new_value: the new occupancy values
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            raise Exception('Point is outside map boundary')

        self.data[y_index][x_index] = new_value

    def set_data(self, point, new_value):
        """
        Set the occupancy value of the given point.
        :param point: a 2D point (x, y) in meters
        :param new_value: the new occupancy value
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        self.set_data_idx((x_index, y_index), new_value)

    def is_inside_idx(self, point_idx):
        """
        Check whether the given point is inside the map.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is inside the map, false otherwise
        """
        x_index, y_index = point_idx
        if x_index < 0 or y_index < 0 or x_index >= self.dim_cells[0] or y_index >= self.dim_cells[1]:
            return False
        else:
            return True

    def is_inside(self, point):
        """
        Check whether the given point is inside the map.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is inside the map, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_inside_idx((x_index, y_index))

    def is_occupied_idx(self, point_idx):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point_idx: a point (x, y) in data array
        :return: True if the given point is occupied, false otherwise
        """
        x_index, y_index = point_idx
        if self.get_data_idx((x_index, y_index)) >= self.occupancy_threshold:
            return True
        else:
            return False

    def is_occupied(self, point):
        """
        Check whether the given point is occupied according the the occupancy threshold.
        :param point: a 2D point (x, y) in meters
        :return: True if the given point is occupied, false otherwise
        """
        x, y = point
        x_index, y_index = self.get_index_from_coordinates(x, y)

        return self.is_occupied_idx((x_index, y_index))

    def get_index_from_coordinates(self, x, y):
        """
        Get the array indices of the given point.
        :param x: the point's x-coordinate in meters
        :param y: the point's y-coordinate in meters
        :return: the corresponding array indices as a (x, y) tuple
        """
        x_index = int(round(x/self.cell_size))
        y_index = int(round(y/self.cell_size))

        return x_index, y_index

    def get_coordinates_from_index(self, x_index, y_index):
        """
        Get the coordinates of the given array point in meters.
        :param x_index: the point's x index
        :param y_index: the point's y index
        :return: the corresponding point in meters as a (x, y) tuple
        """
        x = x_index*self.cell_size
        y = y_index*self.cell_size

        return x, y

    def plot(self, alpha=1, min_val=0, origin='lower'):
        """
        plot the grid map
        """
        plt.imshow(self.data, vmin=min_val, vmax=1, origin=origin, interpolation='none', alpha=alpha)
        plt.draw()

    @staticmethod
    def from_png(filename = '/home/sumslinux/dd2419_ws/src/localization/maps/2D map.png', cell_size = 0.1):
        """
        Create an OccupancyGridMap from a png image
        :param filename: the image filename
        :param cell_size: the image pixel size in meters; default is 0.01 m
        :return: the created OccupancyGridMap
        """
        ogm_data = png_to_ogm(filename, normalized=True)
        ogm_data_arr = np.array(ogm_data)
        ogm = OccupancyGridMap(ogm_data_arr, cell_size)

        return ogm


    # if __name__ == '__main__':
    #     #json_file = '~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json'
    #     json_file = '/home/akanshu/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json'
    #     json_dict = read_json(json_file)
    #     grid, offset = create_grid(json_dict, grid_res=10)
    #     grid = draw_walls(json_dict, grid, offset, grid_res=10)
        

        
    #     cv2.imshow("2D map", img)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()
        

def create_grid(json_dict, grid_res=10):
    # grid_res is the number of rows/columns per meter => 10 means that each cell represents 1 dm x 1 dm
    offset = json_dict['airspace']['min'][:2]
    #print(offset)
    max_point = json_dict['airspace']['max'][:2]
    
    x_len = max_point[0] - offset[0]
    y_len = max_point[1] - offset[1]

    # Adding an extra cell in order to take care of inflated obstacles
    ## such that goal or start position is not out of boundary
    # x_add_value = (x_len)/(grid_res)
    # y_add_value = (y_len)/(grid_res)

    # x_len = x_len + x_add_value
    # y_len = y_len + y_add_value

    grid = np.zeros((int(x_len*grid_res), int(y_len*grid_res)), dtype=np.byte)

    return grid, np.asarray(offset, dtype=np.float64)



def draw_walls(json_dict, grid, offset, grid_res=100):
    # line_res is how many points per meter there are to be between the start and the stop points of a wall
    line_res = 10 * grid_res
    for  d in json_dict['walls']:
        start = np.array(d['plane']['start'][:2]) - offset
        stop = np.array(d['plane']['stop'][:2]) - offset
        # start = start - 0.1 # Converting straight wall into a diagonal wall
        # stop = stop + 0.1 # Converting straight wall into a diagonal wall
        #print(start)
        #print(stop)

        n_points = int(np.linalg.norm(start - stop) * line_res)
        #print(n_points)
        for point in np.linspace(start, stop, n_points):
            floored_point = np.floor(point * (grid_res-1)).astype(dtype=np.int) # The floor of the scalar x is the largest integer i, such that i <= x. 
            #print(floored_point)
            grid[floored_point[0]][floored_point[1]] = 1
            
    return grid

# def grid_index_to_world_coord(self, index, offset, grid_res):
#     index = np.asarray(index, dtype=np.float64)
#     offset = np.asarray(offset, dtype=np.float64)
    
#     return (index / grid_res) + offset

# def world_coord_to_grid_index(self, coord, offset, grid_res):
#     coord = np.asarray(coord, dtype=np.float64)
#     offset = np.asarray(offset, dtype=np.float64)
    
#     return np.floor((coord - offset) * grid_res)

def read_json(path = '/home/sumslinux/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json'):
    with open(path) as f:
        json_dict = json.load(f)

    grid, offset = create_grid(json_dict, grid_res=10)
    grid = draw_walls(json_dict, grid, offset, grid_res=10)
    
    # Plot the grid as an image just to make sure it works
    img = np.zeros(tuple(list(grid.shape) + [3]))
    img[:,:,0] = grid * 255
    img[:,:,1] = grid * 255
    img[:,:,2] = grid * 255
    #cv2.imshow("2D map", img)
    cv2.imwrite('/home/sumslinux/dd2419_ws/src/DD2419-PRAS/localization/maps/2D map.png',img) 
    print("map is printed")
    ogm_from_json = OccupancyGridMap.from_png('/home/sumslinux/dd2419_ws/src/DD2419-PRAS/localization/maps/2D map.png', 0.1)

    return ogm_from_json
