#!/usr/bin/env python

import csv
import numpy as np
import math
import rospy
import rosnode
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from binary_map_4n import path_planning_algo
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from os.path import expanduser
import json

class Trajectorytracking:

    def __init__(self):
        
        # Read trajectory_tracking message topic
        # param1 = rospy.search_param("trajectory_tracking_message_topic")
        # self.trajectory_tracking_msg_topic = rospy.get_param(param1)
        self.trajectory_tracking_topic = '/cf1/cmd_position'

        # Read pose_filtered message topic
        # param2 = rospy.search_param("pose_filtered_message_topic")
        # self.pose_filtered_msg_topic = rospy.get_param(param2)
        self.pose_filtererd_topic = '/pose_filtered'

        self.initial_pose_topic = '/pose_filtered'

        # Initialize callback variables
        self.pose_filtered_msg = None

        # # Initialize class variables
        self.dictgoal = []
        # x_inital = 0
        # y_initial = 0
        self.x_val = 0
        self.y_val = 0
        self.z_val = 0
        self.yaw = 0
        self.cumulative_angle = 0
        self.position_pub = PoseStamped
        self.state = 0
        self.map_odom_yaw = 0
        # Establish publisher of to publish posestamped messages of path
        self.pub = rospy.Publisher(self.trajectory_tracking_topic, Position, queue_size=10)

        # Establish subscription to control message
        rospy.Subscriber(self.pose_filtererd_topic, PoseWithCovarianceStamped, self.pos_filtered_callback)
        rospy.sleep(1)
        # self.init_pose_subscriber = rospy.Subscriber(self.initial_pose_topic, PoseWithCovarianceStamped, self.init_pose)
        # rospy.sleep(0.5)
        # Initialize listener for estimated pose of vehicle in map frame
        self.tf_buf = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buf)

    def dict_goal(self):        # Add capability of input path

        pathcsv = expanduser('~')
        pathcsv += '/dd2419_ws/src/DD2419-PRAS/localization/scripts/Pathcsv'
        with open(pathcsv, 'rb') as csv_file:
            csv_reader = csv.reader(csv_file)
            next(csv_reader)
            for line in csv_reader:
                line[0] = float(line[0])
                line[1] = float(line[1])
                self.dictgoal.append(line)
        # print self.dictgoal[0][1]


    def distance(self, myposex = 0, myposey = 0 , myposez = 0, goalx = 0, goaly = 0,  goalz = 0.4, threshold = 0.15):
        
        # myposez = 0.4
        dist_bool = False
        distance = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2 + (myposez-goalz)**2)
        if distance < threshold:
            dist_bool = True
        else:
            dist_bool = False
        print("distance is",distance)
        print("distance bool is", dist_bool)
        print("Threshold is", threshold)
        return dist_bool

    def angular_distance(self, target_angle = 360, threshold = 5):
        angle_reached = False
        angular_difference = target_angle - self.cumulative_angle
        if angular_difference >= 0:
            if angular_difference <= threshold:
                angle_reached = True
            else:
                angle_reached = False
        elif angular_difference < 0:
            if -angular_difference <= -threshold:
                angle_reached = True
            else:
                angle_reached = False
        
        print("angular_difference is",angular_difference)
        print("angle_reached bool is", angle_reached)
        print("Threshold is", threshold)
        return angle_reached


    def init_pose(self):          # Remove the subscription
        patharray = []
        pathxarray = []
        pathyarray = []

        pose = [0,0]  
        for i in range(0,10):
            pose[0] = round(self.x_val,3)
            pose[1] = round(self.y_val,3)
            patharray.append(pose)
            pathxarray.append(pose[0])
            pathyarray.append(pose[1])
            
        x_inital = round((np.mean(pathxarray)),4)
        y_initial = round((np.mean(pathyarray)),4)
        
        origin = Position()
        origin.header.frame_id = 'cf1/odom'
        origin.x = x_inital
        origin.y = y_initial
        origin.z = 0.4
        origin.yaw = 0
        # self.position_pub = origin
        self.pub.publish(origin)

        rate = rospy.Rate(5)
        distance_bool = self.distance( origin.x, origin.y, self.z_val, origin.x, origin.y, origin.z, 0.025)
        while distance_bool is not True:
            if not rospy.is_shutdown():
                print ("in while loop init_pose")
                self.pub.publish(origin)
                print("Origin x is",origin.x)
                print("Origin y is",origin.y)
                print("Origin z is",origin.z)

                rate.sleep()
                distance_bool = self.distance( origin.x, origin.y, self.z_val, origin.x, origin.y, origin.z, 0.025)
            else:
                break
        
        # return distance_bool, goal_odom_msg.yaw

        # rate = rospy.Rate(5)

        # if not( self.z_val >= 0.375 and self.z_val <= 0.425):
        #     print("self.z_val in take off is", self.z_val)
        #     self.pub.publish(origin)
        #     # rate.sleep()

        rospy.loginfo("Successful take off")
        # return origin       # Returning in order to not have a sudden break after the function exits
        
    def rotation(self, target_angle = 360, angle_step = 5, threshold = 0): # Default direction is counter clockwise
        
        if (self.cumulative_angle + threshold >= 360):
            self.cumulative_angle = self.cumulative_angle - 360
        elif (self.cumulative_angle -threshold <= -360):
            self.cumulative_angle = self.cumulative_angle + 360
        
        # self.cumulative_angle should be in the range [-360, 360]
        if target_angle <= 0:
            dir = 'ccw'
        else: 
            dir = 'cw'

        if dir is 'ccw':        # Positive Angle
            target_angle-= self.cumulative_angle
            if target_angle > 360:
                target_angle = 360 - target_angle
            elif target_angle < -360:
                target_angle = target_angle + 360
            # if target_angle >= 360:
            #     target_angle = target_angle -360
            # elif target_angle <= -360:              # Shouldn't be possible; fail safe
            #     target_angle = target_angle + 360
            angle_step = -angle_step                 # Increment of angle in positive sense

        elif dir is 'cw':
            target_angle += self.cumulative_angle
            if target_angle > 360:
                target_angle = 360 - target_angle
            elif target_angle < -360:
                target_angle = target_angle + 360

            # if target_angle >= 360:
            #     target_angle = target_angle -360
            # elif target_angle <= -360:              # Shouldn't be possible; fail safe
            #     target_angle = target_angle + 360

            angle_step = angle_step                # Increment of angle in negative sense
        else:
            rospy.loginfo("Angle dir is not correct")

        # target_angle should be in the range [-360, 360]
        
        rotation_msg = Position()
        rotation_msg.header.frame_id = 'cf1/odom'
        rate = rospy.Rate(5)
        angle_reached_bool = self.angular_distance(target_angle, threshold)
        while angle_reached_bool is not True:
            if not rospy.is_shutdown():
                
                print ("in while loop rotation")
                # rotation_msg.x = self.x_val
                # rotation_msg.y = self.y_val
                if (self.z_val>= 0.375 and self.z_val <= 0.425):
                    rotation_msg.z = self.z_val
                else:
                    rotation_msg.z = 0.4

                self.cumulative_angle += angle_step
                rotation_msg.yaw = self.cumulative_angle
                self.pub.publish(rotation_msg)
                rate.sleep()
                print("Cumulative angle is", self.cumulative_angle)
                angle_reached_bool = self.angular_distance(target_angle, threshold)
            else:
                # rospy.sleep(0.05)
                break
                
            
        # return distance_bool, goal_odom_msg.yaw

        # for i in range(int(self.cumulative_angle), int(angle_step), int(target_angle)):
        #     print(i)
        #     print(self.cumulative_angle)
        #     print(target_angle)
        #     print("in rotation")
        #     rotation_msg.x = self.x_val
        #     rotation_msg.y = self.y_val
        #     if (self.z_val>= 0.375 and self.z_val <= 0.425):
        #         rotation_msg.z = self.z_val
        #     else:
        #         rotation_msg.z = 0.4

        #     self.cumulative_angle += angle_step
        #     rotation_msg.yaw = self.cumulative_angle
        #     self.pub.publish(rotation_msg)
        #     rate.sleep()                                # Check the performance with this; might break the code
        
        # return rotation_msg
        
    def landing(self, alt_desired = 0.15):

        print ('in landing')
        landing_cmd = Position()
        landing_cmd.header.frame_id = 'cf1/odom'
        
        if self.z_val < 0.15:
            rospy.loginfo("Already less than 0.15. Exiting landing")
        
        alt_current = self.z_val
        rate = rospy.Rate(5)
        while not( alt_current <= alt_desired):
            alt_current = self.z_val    
            alt_current -= 0.1
            landing_cmd.x = self.x_val
            landing_cmd.y = self.y_val
            landing_cmd.yaw = self.yaw
            landing_cmd.z = alt_current
            rate.sleep()
        
        # rosnode.kill_nodes(['trajectory_tracking'])
        return landing_cmd       # Returning in order to not have a sudden break after the function exits

    def kill_node(self, node_name):
        success, fail = rosnode.kill_nodes([node_name])
        return node_name in success

    def goalpub(self, goal_x = 0, goal_y = 0, offset = 0.0):    # Change offset to 0 while localizing 

        goal_map = PoseStamped()
        goal_map.header.stamp = rospy.Time.now()
        goal_map.header.frame_id = "map"
        goal_map.pose.position.x = goal_x
        goal_map.pose.position.y = goal_y
        goal_map.pose.position.z = 0.4        

        if not self.tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
            rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal_map.header.frame_id)
            return

        goal_odom = self.tf_buf.transform(goal_map, 'cf1/odom')

        if goal_odom:

            goal_odom_msg = Position()
            goal_odom_msg.header.stamp = rospy.Time.now()
            goal_odom_msg.header.frame_id = "cf1/odom"
            goal_odom_msg.x = goal_odom.pose.position.x + offset
            goal_odom_msg.y = goal_odom.pose.position.y + offset
            goal_odom_msg.z = 0.4
            goal_odom_msg.yaw = self.cumulative_angle                # Check This!

        rate = rospy.Rate(5)
        distance_bool = self.distance(self.x_val, self.y_val, self.z_val, goal_odom_msg.x, goal_odom_msg.y, goal_odom_msg.z, 0.15)  # care for z
        while distance_bool is not True:
            if not rospy.is_shutdown():

                print ("in while loop pubgoal")
                self.pub.publish(goal_odom_msg)
                rate.sleep()
                distance_bool = self.distance(self.x_val, self.y_val, self.z_val, goal_odom_msg.x, goal_odom_msg.y, goal_odom_msg.z, 0.15)  

            else:
                break

        return distance_bool, goal_odom_msg.yaw
            

    def pos_filtered_callback(self, cf1_filtered_pose):
        
        self.pose_filtered_msg = cf1_filtered_pose
        self.x_val = round(self.pose_filtered_msg.pose.pose.position.x, 3)
        self.y_val = round(self.pose_filtered_msg.pose.pose.position.y, 3)
        self.z_val = round(self.pose_filtered_msg.pose.pose.position.z, 3)
        (roll, pitch, yaw) = euler_from_quaternion([self.pose_filtered_msg.pose.pose.orientation.x, self.pose_filtered_msg.pose.pose.orientation.y, self.pose_filtered_msg.pose.pose.orientation.z, self.pose_filtered_msg.pose.pose.orientation.w],axes='rzxy') #output in radians, Check the axis!!!! rzyx
            
        self.yaw = round(yaw,3)

        # Converting into degrees to subtract from map orientation angles which are in euler i.e in degrees
        self.yaw = math.degrees(self.yaw)

        # print("z_val is", self.z_val)

    def state_machine(self, x_goal = 0, y_goal = 0):
        print("x_goal is",x_goal)
        if x_goal!= 100 and y_goal!=100:
            reached_goal, self.map_odom_yaw = self.goalpub(x_goal, y_goal)
            print("Reached goal is ", reached_goal)
        
        elif x_goal == 100 and y_goal == 100:
            desired_angle = -180 - self.map_odom_yaw
            self.rotation(desired_angle)

            print("The drone has reached the goal and now spinning 360")
            if self.cumulative_angle >= 0:   # Spinning around the goal
                self.rotation(-360)
                rospy.loginfo("Completed Goal Rotation -360")
            else:
                self.rotation(360)
                rospy.loginfo("Completed Goal Rotation 360")

    def main(self):
        if not rospy.is_shutdown():
            
            self.init_pose()
            rospy.loginfo("Successful init_pose")
            # rospy.sleep(0.1)

            self.rotation()     # Initial rotation

            self.dict_goal()
            rospy.loginfo("Successful dict_goal")


            for i in range(len(self.dictgoal) - 1):
                if self.z_val > 0.1 and self.z_val < 0.7:
                    self.state_machine(self.dictgoal[i][0], self.dictgoal[i][1])
                print("Hey")
                    # self.kill_node(node_name = 'trajectory_tracking')

                if i == (len(self.dictgoal) - 1):
                    self.landing()
                    print("Landing")
            
            self.kill_node(node_name = 'trajectory_tracking')

if __name__ == '__main__':

    rospy.init_node('trajectory_tracking', anonymous=True)
    rospy.loginfo("Successful initilization of trajectory tracking node")
    quad = Trajectorytracking()
    quad.main()
    # rospy.spin()