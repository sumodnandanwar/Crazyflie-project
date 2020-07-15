#!/usr/bin/env python

import csv
import pandas as pd
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.msg import Position
from binary_map_4n import path_planning_algo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from os.path import expanduser
import json

class Pathplanning:

    def __init__(self, start= None, goal = None, algo = 'a-star'):
        
        self.start = start
        self.goal = goal

        # Read path_planning message topic
        # param1 = rospy.search_param("path_planning_message_topic")
        # self.path_planning_msg_topic = rospy.get_param(param1)
        self.path_planning_msg_topic = '/planned_path'

        # Initialize callback variables
        # self.goal_msg = None

        # # Initialize class variables
        self.goal_msg = None
        self.start_msg = None
        self.path_planning_msg = None


        # Establish publisher of to publish posestamped messages of path
        self.pub = rospy.Publisher(self.path_planning_msg_topic, Path, queue_size=10)

        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    def start_position(self):
        
        self.start_msg = PoseStamped()
        self.start_msg.header.frame_id = 'image'
        trans = None
        trans_bool = False
        target_frame = 'image'
        # 22:00 13.05.20 Changing to base link instead of odom
        try:
            trans = self.tfBuffer.lookup_transform(target_frame,'cf1/base_link',rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Lookup transfrom from odom to map is available')
            trans_bool = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Failure of lookup transfrom from odom to map')
            trans_bool = False
            # self.start_msg.pose.position.x = -0.25          # Default Position in our created world
            # self.start_msg.pose.position.y = 0.4
            

        if trans != None:
            self.start_msg.header.stamp = rospy.Time.now()
            self.start_msg.pose.position = trans.transform.translation
            self.start_msg.pose.orientation = trans.transform.rotation
            # self.pub_start.publish(self.start_msg)            # Don't know why?????????
        
        return trans_bool


    def get_goal_position(self):      # Run this to get the positions 

        path = expanduser('~')
        path += '/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json'
        with open(path, 'rb') as f:
            world = json.load(f)
        # jsonMarker = [m for m in world['markers']]
        # print( jsonMarker)
        # print (jsonMarker[0]['pose']['position'][2])

        #Returns everything in roadsigns
        self.jsonsign = [m for m in world['roadsigns']]
        self.goal_to_sign()
        self.start_to_end()

    def goal_to_sign(self):
        x = 0
        y = 0
        goal_sign = 0
        self.dict_signs = {}
        for m in self.jsonsign:
            goal_sign = goal_sign
            goalsign1 = [x,y]
            if m['pose']['position'][0] < 0:
                goalsign1[0] = m['pose']['position'][0] + 0.25
            elif m['pose']['position'][0] > 0:
                goalsign1[0] = m['pose']['position'][0] - 0.25
            goalsign1[1] = m['pose']['position'][1]
            self.dict_signs[goal_sign] = goalsign1
            goal_sign += 1

    def start_to_end(self):
        # initialize = 0
        no_signs = len(self.dict_signs)+1
        self.waypoints = {}
        for i in range(no_signs):
            # From start to first goal
            if i == 0:
                start = [self.start_msg.pose.position.x, self.start_msg.pose.position.y]
                end = [self.dict_signs[i][0],self.dict_signs[i][1]]
                self.waypoints[i] = start,end
                
            # Waypoint between goals
            elif i == 1:
                for ii in range(no_signs-2):
                    start = [self.dict_signs[ii][0],self.dict_signs[ii][1]]
                    end = [self.dict_signs[ii+1][0],self.dict_signs[ii+1][1]]
                    self.waypoints[ii+1] = start,end
                    
            # From last goal to start back
            elif i == no_signs-1:
                start = [self.dict_signs[i-1][0],self.dict_signs[i-1][1]]
                end = [self.start_msg.pose.position.x, self.start_msg.pose.position.y]
                self.waypoints[i] = start,end
        
        # print(self.waypoints)
                

    def path_planning_start_end(self):    def publish_path(self):

        rate = rospy.Rate(1) # 1 hz
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        
        while not rospy.is_shutdown():

            self.path_planning_msg = PoseStamped()
        
            for i in range(len(self.path_complete)):
                self.x, self.y = self.path_complete[i]
                #print ('x is',type(self.x))
                #print ('y is',self.y)                
                self.path_planning_msg.header.frame_id = 'map'
                self.path_planning_msg.header.stamp = rospy.Time.now()

                self.path_planning_msg.pose.position.x = (self.y*(-0.1))+0.5 # # Converting into meter because rviz grid/ drone world works in meter; our world res is in dm and origin is 0.5 instead of 10
                self.path_planning_msg.pose.position.y = (self.x*(0.1))-0.5 # # Converting into meter because rviz grid/ drone world works in meter
                self.path_planning_msg.pose.position.z = 0.4 # Static value for z since 2D path planning
                    
                self.path_planning_msg.pose.orientation.x = 0 # # Didn't Convert into meter because it is static
                self.path_planning_msg.pose.orientation.y = 0 # # 
                self.path_planning_msg.pose.orientation.z = 0 # # 
                self.path_planning_msg.pose.orientation.w = 1 # # 

                #print(self.path)
                path.poses.append(self.path_planning_msg)
                # print(path)

            self.pub.publish(path)

            
            rate.sleep()

        # rate = rospy.Rate(10) # 10hz
        
        # while not rospy.is_shutdown():
        # print("In path_planning_start_end")
        self.path_complete = []
        self.path_waypoint = []
        path_break = [(100,100)] # Identifier for path reached
        for i in range (len(self.waypoints)):
            self.start = self.waypoints[i][0]
            self.goal = self.waypoints[i][1]
        
            try:
                self.path_waypoint = self.path_planning(self.start, self.goal) # return the path px from A-star
                rospy.loginfo("Path found by A-star")
                self.path_complete.extend(path_break)
                self.path_complete.extend(self.path_waypoint)

            except:
                rospy.loginfo("Path not found by A-star")
            
            self.path_complete = self.path_complete[1:-1]
            # self.path_complete.extend(self.path_waypoint)
            # self.path_complete.extend(path_break)
        
        print(self.path_complete)
        df = pd.DataFrame(self.path_complete, columns=['x', 'y'])

        df['x'] = df['x'].apply(lambda x: (x*0.1))
        df['x'] = df['x'].apply(lambda x: x-0.5)
        df['y'] = df['y'].apply(lambda x: (x*0.1))
        df['y'] = df['y'].apply(lambda x: x-0.5)
        temp = pd.DataFrame(df['x'])
        df['x'] = -df['y']
        df['y'] = temp

        df['x'] = df['x'].apply(lambda x: round(x,2))
        df['y'] = df['y'].apply(lambda y: round(y,2))


        pathcsv = expanduser('~')
        pathcsv += '/dd2419_ws/src/DD2419-PRAS/localization/scripts/Pathcsv'
        df.to_csv(pathcsv, sep=',',index=False)

        # print(df)

        # print(self.path_complete)
        self.publish_path()
        
            # rate.sleep()


    def path_planning(self, start=[-0.25, 0.5], end = [0, 0], algo = 'a-star', plot= False):
        print ("start is :", start)
        print ("end is :", end)
        
        # while not rospy.is_shutdown():
            
        start = ((0.5 + start[1]), (0.5 - start[0]))   # Subtracting And Adding 0.5/ centre of the image frame to get map frame points into image frame
        end = ((0.5 + end[1]), (0.5 - end[0]))            # Subtracting And Adding 0.5/ centre of the image frame to get map frame points into image frame

        start = (start[0], start[1])               # Since Map and Image X and Y are different
        end = (end[0], end[1])    

        print ("start is :", start)
        print ("end is :", end)

        if algo == 'a-star':
            path, path_px = path_planning_algo(start, end, algo, plot)
            #return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)

            #path_px = path_px 
            # print ("Path px is ",path_px)
            #self.pub.publish("Sucess")
            # print ("Sucess!!!!!!!!!!!!!")
        else:
            return 0
                        
        return path_px 

    def publish_path(self):

        rate = rospy.Rate(1) # 1 hz
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = rospy.Time.now()
        
        while not rospy.is_shutdown():

            self.path_planning_msg = PoseStamped()
        
            for i in range(len(self.path_complete)):
                self.x, self.y = self.path_complete[i]
                #print ('x is',type(self.x))
                #print ('y is',self.y)                
                self.path_planning_msg.header.frame_id = 'map'
                self.path_planning_msg.header.stamp = rospy.Time.now()

                self.path_planning_msg.pose.position.x = (self.y*(-0.1))+0.5 # # Converting into meter because rviz grid/ drone world works in meter; our world res is in dm and origin is 0.5 instead of 10
                self.path_planning_msg.pose.position.y = (self.x*(0.1))-0.5 # # Converting into meter because rviz grid/ drone world works in meter
                self.path_planning_msg.pose.position.z = 0.4 # Static value for z since 2D path planning
                    
                self.path_planning_msg.pose.orientation.x = 0 # # Didn't Convert into meter because it is static
                self.path_planning_msg.pose.orientation.y = 0 # # 
                self.path_planning_msg.pose.orientation.z = 0 # # 
                self.path_planning_msg.pose.orientation.w = 1 # # 

                #print(self.path)
                path.poses.append(self.path_planning_msg)
                # print(path)

            self.pub.publish(path)

            
            rate.sleep()

if __name__ == '__main__':

    trans_bool = False
    rospy.init_node('Pathplanning', anonymous=True)
    rospy.loginfo("Successful initilization of path planning node")
    quad = Pathplanning()
    while trans_bool is not True:
        if not rospy.is_shutdown():
            trans_bool = quad.start_position()
            print(trans_bool)
        else:
            break
    if trans_bool is True:
        quad.get_goal_position()
        quad.path_planning_start_end()
    rospy.spin()
    #rospy.init_node('navgoal3')