#!/usr/bin/env python   

import sys
import math
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
#import keyboard  # using module keyboard
from numpy import genfromtxt
from visualization_msgs.msg import MarkerArray

goal_bool = 0


def get_data(file = "/home/akanshu/dd2419_ws/src/localization/maps/point.csv"):
    
    x = []
    y = []
    my_data = genfromtxt(file, delimiter=',')
    for i in range (len(my_data)):
        
        x.append(my_data[i,5])
        y.append(my_data[i,4])

    x = np.array(x)
    y = np.array(y)
    start_end = np.stack(np.meshgrid(x, y), -1).reshape(-1, 2)
    print(start_end)
    return(start_end)

def test_algo(data, n=5):
    global goal_bool
    goal_succes = []
    #data_random = np.random.shuffle(data)
    #print(data_random)
    # Here n is the number of tests to be performed on the algo
    for i in range(n):
        x_start = data[i][0]
        y_start = data[i][1]
        x_goal = data[i+1][0]
        y_goal = data[i+1][1]
        current_pos(x_start, y_start)
        publish_goal(x_goal,y_goal)
        rospy.sleep(5)
        
        if goal_bool == 0:
            rospy.loginfo("Path not found")
        elif goal_bool == 1:
            rospy.loginfo("Path found")
        else:
            rospy.loginfo("goal_bool value other than 0 or 1 found")

        goal_succes.append(goal_bool)
        goal_bool = 0
        # start = tuple((x_start, y_start))
        # goal = tuple((x_goal, y_goal))
        # print(start)
        return goal_succes
        print(goal_bool)

# def get_keystroke():

#     var1 = 'null'
#     var2 = 'null'
#     count_goal = 0
#     #print("Variable is: ", var)
#     while (var1 != 'exit'):  # making a loop
#         var1 = raw_input("Please type exit for exiting or new to a pass a new input: ") # if nothing is pressed

#         if var1 == ('new'):  # if new is typed
#             var2 = raw_input("Please type pose for current position input and goal for goal position input: ") # if nothing is pressed

#             if var2 == ('pose'):
#                 print('Please enter a new cuurent position')
#                 current_pos()
#                 print("Successfully published above current position") # Display completion and accept new input

#             elif var2 == ('goal'):
#                 count_goal+=1 
#                 print('Please enter a new goal position')
#                 if count_goal == 1:

#                     publish_goal()
#                     print("Successfully published above goal position on goal topic ") # Display completion and accept new input

#                 else:

#                     publish_goal()
#                     print("Successfully published above goal position on Overwrite goal topic") # Display completion and accept new input

#             else:
#                 print("Please enter valid input") # Re-enter the input


#         elif var1 == ('exit'):  # if exit is typed, exit!
#             print('Exiting')
#             break

#         else:
#             print("Please enter valid input") # Re-enter the input

def current_pos(x_current, y_current):

    current_pos = PoseStamped()
    pub_current_pos = rospy.Publisher('/current_pose', PoseStamped, queue_size = 2)

    current_pos.header.stamp = rospy.Time.now()
    current_pos.header.frame_id = "map"

    current_pos.pose.position.x = x_current
    current_pos.pose.position.y = y_current

    # current_pos.pose.position.x = float(input("Enter x value: "))
    # current_pos.pose.position.y = float(input("Enter y value: "))
    #current_pos.pose.position.z = float(input("Enter z value: "))
    current_pos.pose.position.z = 0

    # current_pos.pose.orientation.x = float(input("Enter x value of quaternion: "))
    # current_pos.pose.orientation.y = float(input("Enter y value of quaternion: "))
    # current_pos.pose.orientation.z = float(input("Enter z value of quaternion: "))
    # current_pos.pose.orientation.w = float(input("Enter w value of quaternion: "))

    current_pos.pose.orientation.x = 1 # As shared during conversation
    current_pos.pose.orientation.y = 0
    current_pos.pose.orientation.z = 0
    current_pos.pose.orientation.w = 0

    pub_current_pos.publish(current_pos)
    #rospy.loginfo('goal is', goal)


def publish_goal(x_goal, y_goal):

    goal = PoseStamped()
    pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size = 2)

    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"

    goal.pose.position.x = x_goal
    goal.pose.position.y = y_goal

    # goal.pose.position.x = float(input("Enter x value: "))
    # goal.pose.position.y = float(input("Enter y value: "))
    goal.pose.position.z = 0
    #goal.pose.position.z = float(input("Enter z value: "))


    goal.pose.orientation.x = 1 # As shared
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = 0
    goal.pose.orientation.w = 0

    # goal.pose.orientation.x = float(input("Enter x value of quaternion: "))
    # goal.pose.orientation.y = float(input("Enter y value of quaternion: "))
    # goal.pose.orientation.z = float(input("Enter z value of quaternion: "))
    # goal.pose.orientation.w = float(input("Enter w value of quaternion: "))

    pub_goal.publish(goal)
    #rospy.loginfo('goal is', goal)

# def publish_overwritegoal():

#     overwritegoal = PoseStamped()
#     pub_overwritegoal = rospy.Publisher('/adeye/overwriteGoal', PoseStamped, queue_size = 2)

#     overwritegoal.header.stamp = rospy.Time.now()
#     overwritegoal.header.frame_id = "map"

#     overwritegoal.pose.position.x = float(input("Enter x value: "))
#     overwritegoal.pose.position.y = float(input("Enter y value: "))
#     overwritegoal.pose.position.z = 0
#     #goal.pose.position.z = float(input("Enter z value: "))


#     overwritegoal.pose.orientation.x = 1 # As shared
#     overwritegoal.pose.orientation.y = 0
#     overwritegoal.pose.orientation.z = 0
#     overwritegoal.pose.orientation.w = 0

#     # goal.pose.orientation.x = float(input("Enter x value of quaternion: "))
#     # goal.pose.orientation.y = float(input("Enter y value of quaternion: "))
#     # goal.pose.orientation.z = float(input("Enter z value of quaternion: "))
#     # goal.pose.orientation.w = float(input("Enter w value of quaternion: "))

#     pub_overwritegoal.publish(overwritegoal)
#     #rospy.loginfo('goal is', goal)

def path_msg_callback(goal_msg):
    global goal_bool
    goal_bool = 1


def main():
    # rospy.Rate(20)
    #while not rospy.is_shutdown():
    #     if rgoal:
    #         publish_goal(rgoal)
    #     # else:
    #     #     print('Final pose (x,y,z,yaw(deg)) = ')
    #     #     kgoal = input()
    #     #     pgoal = kgoal.split(',') 
    #     #     pgoal[0] = float(pgoal[0])
    #     #     pgoal[1] = float(pgoal[1])
    #     #     pgoal[2] = float(pgoal[2])
    #     #     pgoal[3] = float(pgoal[3])
    #     #     pub_goal.publish(pgoal)
    #     #     rospy.loginfo('pgoal is', pgoal)
    # rospy.sleep(1)
        rospy.Subscriber('/global_waypoints_rviz', MarkerArray, path_msg_callback)
        rospy.sleep(2)
        data = get_data()
        goal_success =test_algo(data)
        #rospy.init_node('my_map_goal')
        #get_keystroke()
        #sub_goal = rospy.Subscriber('', , goal_callback)

if __name__ == '__main__':
    main()