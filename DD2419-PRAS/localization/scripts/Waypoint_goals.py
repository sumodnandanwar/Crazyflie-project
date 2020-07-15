#!/usr/bin/env python

import math
import rospy
import json
from os.path import expanduser

dict_signs = {}
waypoints = {}

def goal_to_sign(jsonsign):
    x = 0
    y = 0
    goal_sign = 0
    for m in jsonsign:
        goal_sign = goal_sign
        goalsign1 = [x,y]
        if m['pose']['position'][0] < 0:
            goalsign1[0] = m['pose']['position'][0] + 0.25
        elif m['pose']['position'][0] > 0:
            goalsign1[0] = m['pose']['position'][0] - 0.25
        goalsign1[1] = m['pose']['position'][1]
        dict_signs[goal_sign] = goalsign1
        goal_sign += 1
    # print dict_signs
    return dict_signs


def start_to_end():
    # initialize = 0
    no_signs = len(dict_signs) + 1
    # print no_signs
    for i in range(no_signs):
        # From start to first goal
        if i == 0:
            start = [0,0]
            end = [dict_signs[i][0],dict_signs[i][1]]
            waypoints[i] = start,end
            
        # Waypoint between goals
        elif i == 1:
            for ii in range(no_signs-2):
                start = [dict_signs[ii][0],dict_signs[ii][1]]
                end = [dict_signs[ii+1][0],dict_signs[ii+1][1]]
                waypoints[ii+1] = start,end
                
        # From last goal to start back
        elif i == no_signs-1:
            start = [dict_signs[i-1][0],dict_signs[i-1][1]]
            end = [0,0]
            waypoints[i] = start,end
    # print waypoints
    return waypoints

# print waypoints

rospy.init_node('Start_and_end_goals')
# start_pub = rospy.Publisher('Waypoint_goal',String , queue_size=2)
# end_pub = rospy.Publisher('end_goal', Position, queue_size=5)

def main():
    global jsonsign
    # rate = rospy.Rate(10)  # Hz
    path = expanduser('~')
    path += '/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json'
    with open(path, 'rb') as f:
        world = json.load(f)
    # jsonMarker = [m for m in world['markers']]
    # print jsonMarker
    # print jsonMarker[0]['pose']['position'][2]
    #Returns everything in roadsigns
    jsonsign = [m for m in world['roadsigns']]
    # while not rospy.is_shutdown():
    goal_to_sign(jsonsign)
    start_to_end()
    # print waypoints
    rospy.spin()

if __name__ == '__main__':
    main()
    print waypoints