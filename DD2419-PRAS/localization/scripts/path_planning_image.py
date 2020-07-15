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
        # path_planning_msg_topic = rospy.get_param(param1)
        path_planning_msg_topic = '/planned_path'

        # Initialize callback variables

        # # Initialize class variables
        self.goal_msg = None
        self.start_msg = None
        self.goal_img = None
        # path_planning_msg = None

        # Establish publisher of to publish posestamped messages of path
        self.pub = rospy.Publisher(path_planning_msg_topic, Path, queue_size=10)

        # Initialize listener for estimated pose of vehicle in image frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)


    def goal_position(self, x_map = -0.1, y_map = 0.4):
        self.goal_img = PoseStamped()
        self.goal_img.header.stamp = rospy.Time.now()
        self.goal_img.header.frame_id = "map"
        self.goal_img.pose.position.x = x_map
        self.goal_img.pose.position.y = y_map
        self.goal_img.pose.position.z = 0.4
        if not self.tfBuffer.can_transform('image', 'map', rospy.Time.now(), rospy.Duration(1.0)):
            rospy.logwarn_throttle(10.0, 'No transform from %s to image' % self.goal_img.header.frame_id)
            return

        self.goal_img = self.tfBuffer.transform(self.goal_img, 'image')
        # print(self.goal_img)


    def start_position(self):
        
        self.start_msg = PoseStamped()
        self.start_msg.header.frame_id = 'image'
        trans_bool = False
        trans = None
        target_frame = 'image'
        try:
            trans = self.tfBuffer.lookup_transform(target_frame,'cf1/base_link',rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Goal Transform: Lookup transfrom from cf1/base_link to image is available')
            trans_bool = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            #trans = self.tfBuffer.lookup_transform(self.image_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Goal transform: Failure of lookup transfrom from cf1/base_link to image')
            trans_bool = False
            # self.start_msg.pose.position.x = -0.25          # Default Position in our created world
            # self.start_msg.pose.position.y = 0.4
            
        if trans != None:
            self.start_msg.header.stamp = rospy.Time.now()
            self.start_msg.pose.position = trans.transform.translation
            self.start_msg.pose.orientation = trans.transform.rotation
        
        return trans_bool


    def get_goal_position(self):      # Run this to get the positions 

        path = expanduser('~')
        path += '/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json'
        with open(path, 'rb') as f:
            world = json.load(f)

        #Returns everything in roadsigns
        self.jsonsign = [m for m in world['roadsigns']]
        self.goal_to_sign()
        self.start_to_end()

    def goal_to_sign(self):
        x = 0
        y = 0
        goal_sign = 0
        offset_collision = 0.26
        self.dict_signs = {}
        for m in self.jsonsign:
            goal_sign = goal_sign
            goalsign1 = [x,y]
            if m['pose']['position'][0] < 0:
                goalsign1[0] = m['pose']['position'][0] + offset_collision
            elif m['pose']['position'][0] > 0:
                goalsign1[0] = m['pose']['position'][0] - offset_collision
            goalsign1[1] = m['pose']['position'][1]

            # Converting to image Frame ################
            self.goal_position(goalsign1[0], goalsign1[1])
            goalsign1[0] = self.goal_img.pose.position.x
            goalsign1[1] = self.goal_img.pose.position.y
            ###############################################
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
                        

    def path_planning_start_end(self):

        self.path_complete = []
        self.path_waypoint = []
        path_break = [(100,100)] # Identifier for path reached

        for i in range (len(self.waypoints)):
            self.start = self.waypoints[i][0]
            self.goal = self.waypoints[i][1]
        
            try:
                self.path_waypoint = self.path_planning(self.start, self.goal) # return the path px from A-star in image frame
                rospy.loginfo("Path found by A-star in image frame")
                # print(self.path_waypoint)
                self.path_waypoint = self.path_x_y_map_list(self.path_waypoint, 0.1) # return the path in map frame
                # print(self.path_waypoint)
                rospy.loginfo("Path found in map frame")
                self.path_complete.extend(self.path_waypoint)
                self.path_complete.extend(path_break)

            except:
                rospy.loginfo("Path not found by A-star")
            
        
        # print(self.path_complete)
        df = pd.DataFrame(self.path_complete, columns=['x', 'y'])

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

        start = (start[0], start[1])               
        end = (end[0], end[1])    

        print ("start is :", start)
        print ("end is :", end)

        if algo == 'a-star':
            path, path_px = path_planning_algo(start, end, algo, plot)
            #return: a tuple that contains: (the resulting path in meters, the resulting path in data array indices)
        else:
            return 0
                        
        return path_px 

    # Converting the list returned by A-star with co-ordinates multiplied by 1/resolution
    def path_x_y_map_list(self, list_path_image, resolution = 0.1):
        
        path_map_list = list_path_image
        x_y_map = PoseStamped()

        if not self.tfBuffer.can_transform('map', 'image', rospy.Time.now(), timeout=rospy.Duration(0.2)):
            rospy.logwarn_throttle(10.0, 'No transform from image to map')
            return

        # Convering into map frame
        for i in range(len(path_map_list)):
            x_y_map.header.frame_id = 'image'
            x_y_map.header.stamp = rospy.Time.now()

            x_map, y_map = path_map_list[i]
            x_y_map.pose.position.x = x_map * resolution    # Multiply by resolution to get it in original resoltion of the map
            x_y_map.pose.position.y = y_map *resolution

            x_y_map = self.tfBuffer.transform(x_y_map, 'map')                       # Transforming into map
            path_map_list[i] = (x_y_map.pose.position.x, x_y_map.pose.position.y)

        return path_map_list

    def publish_path(self):

        rate = rospy.Rate(1) # 1 hz
        path = Path()
        
        while not rospy.is_shutdown():

            path_planning_msg = PoseStamped()
            path_map_msg = PoseStamped()

            for i in range(len(self.path_complete)):
                x_path, y_path = self.path_complete[i]

                path_planning_msg.header.frame_id = 'map'
                path_planning_msg.header.stamp = rospy.Time.now()

                path_planning_msg.pose.position.x = x_path 
                path_planning_msg.pose.position.y = y_path 

                path_planning_msg.pose.position.z = 0.4 
                    
                path_planning_msg.pose.orientation.x = 0 
                path_planning_msg.pose.orientation.y = 0 
                path_planning_msg.pose.orientation.z = 0 
                path_planning_msg.pose.orientation.w = 1 
                
                path_map_msg = self.tfBuffer.transform(path_planning_msg, 'map')

                path.poses.append(path_map_msg)

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
            # print(trans_bool)
        else:
            break
    if trans_bool is True:
        quad.get_goal_position()
        quad.path_planning_start_end()
    rospy.spin()
    
    # rospy.init_node('Pathplanning', anonymous=True)
    # rospy.loginfo("Successful initilization of path planning node")

    # rate = rospy.Rate(1)          # Run every sec
    # quad = Pathplanning()

    # while not rospy.is_shutdown(): 
    #     # trans_bool = False
    #     # while trans_bool is not True:
    #     #     if not rospy.is_shutdown():
    #     trans_bool = quad.start_position()
    #             # print(trans_bool)
    #         # else:
    #         #     break
    #     if trans_bool is True:
    #         quad.get_goal_position()
    #         quad.path_planning_start_end()

    # rate.sleep()