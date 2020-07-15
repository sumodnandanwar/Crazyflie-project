#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/sumslinux/dd2419_ws/src/localization/scripts")
from gridmap import OccupancyGridMap, read_json
import matplotlib.pyplot as plt
from a_star import a_star
from utils import plot_path


def path_planning_algo(start, end, algo= 'a-star', plot =False):
    #return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)

    path= None
    path_px = None

    if algo == 'a-star':
            
        # load the map
        #gmap = OccupancyGridMap.from_png('maps/example_map_binary.png', 0.1)
        #gmap = OccupancyGridMap.from_png('maps/2D map.png', 0.1)
        gmap = read_json()

        # set a start and an end node (in meters)
        # start_node = (360.0, 330.0)
        # goal_node = (285.0, 86.0)

        # start_node = (6.0, 4.25)
        # goal_node = (15.0, 17.0)
    
        start_node = start
        goal_node = end

        # run A*
        path, path_px = a_star(start_node, goal_node, gmap, movement='4N')

        if plot == True:

            gmap.plot()

            if path:
                # plot resulting path in pixels over the map
                plot_path(path_px)
                #print(path_px)
            else:
                print('Goal is not reachable')

                # plot start and goal points over the map (in pixels)
                start_node_px = gmap.get_index_from_coordinates(start_node[0], start_node[1])
                goal_node_px = gmap.get_index_from_coordinates(goal_node[0], goal_node[1])

                plt.plot(start_node_px[0], start_node_px[1], 'ro')
                plt.plot(goal_node_px[0], goal_node_px[1], 'go')

            plt.show()
                
    return path, path_px
