#!/usr/bin/env python
"""
Created on Wednesday Apr 22 17:14:00 2020
@author: Akanshu Mahajan
"""

#import json
import sys
import math
import rospy
import json
import string
import tf2_ros
import tf2_geometry_msgs
#import geometry_msgs.msg
import nav_msgs.msg

from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3
from geometry_msgs.msg import PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover

from std_msgs.msg import Empty
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker

#from binary_map_4n import localization_algo


class Localization:

    def __init__(self):
        
        ## Pull necessary ROS parameters from launch file:
        # Read goal message topic
        #param1 = rospy.search_param("marker_map_message_topic") 
        #self.marker_map_message_topic = rospy.get_param(param1)       # /aruco_map_pose

        # Read Aruco message topic
        # param2 = rospy.search_param("marker_odom_message_topic")
        # self.marker_odom_message_topic = rospy.get_param(param2) #/aruco_odom_pose_tf

        self.marker_odom_message_topic = '/aruco_odom_pose_tf'
        # Initialize callback variables
        #self.marker_map_msg = None
        self.marker_odom_msg = None

        # Initialize class variables
        #self.localization_msg = None

        # Establish subscription to control message
        #rospy.Subscriber(self.marker_map_message_topic, PoseStamped, self.marker_map_msg_callback)
        rospy.Subscriber(self.marker_odom_message_topic, PoseStamped, self.marker_odom_msg_callback)
        
        # Delay briefly for subscriber to find message
        rospy.sleep(2)

        # # Establish publisher of converted Twist message
        # self.pub = rospy.Publisher(self.localization_msg_topic, PoseStamped, queue_size=10)

        # Initilize tf2 broadcaster and transform message
        #self.br = tf2_ros.TransformBroadcaster()
        self.br = tf2_ros.StaticTransformBroadcaster()
        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # def marker_map_msg_callback(self, marker_map_msg):
    #     self.marker_map_msg = marker_map_msg

    def marker_odom_msg_callback(self, marker_odom_msg):
        self.marker_odom_msg = marker_odom_msg

    def marker_map(self, m):

        self.x_map, self.y_map, self.z_map = m['pose']['position']
        self.roll_map, self.pitch_map, self.yaw_map = m['pose']['orientation']       
        

    def diff(self):

        self.x_diff = self.x_map - self.x_odom
        self.y_diff = self.y_map - self.y_odom
        self.z_diff = self.y_map - self.y_odom

        self.roll_diff = self.roll_map - self.roll_odom
        self.pitch_diff = self.pitch_map - self.pitch_odom
        self.yaw_diff = self.yaw_map - self.yaw_odom

        # self.x_diff = self.x_map - self.x_odom
        # self.y_diff = self.y_map - self.y_odom
        # self.z_diff = self.y_map - self.y_odom

        # self.roll_diff = self.roll_map - self.roll_odom
        # self.pitch_diff = 0
        # self.yaw_diff = 0


    def odom_to_map(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'cf1/odom'
        t.transform.translation.x = self.x_diff
        t.transform.translation.y = self.y_diff
        t.transform.translation.z = self.z_diff
        
        (t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w) = quaternion_from_euler(math.radians(self.roll_diff),
                                                     math.radians(self.pitch_diff),
                                                     math.radians(self.yaw_diff),'ryxz')
        
        return t


    def marker_odom(self, marker):
        
        trans = None
        transform = TransformStamped()
        source_frame = "aruco/detected" + str(marker['id'])

        try:
            trans = self.tfBuffer.lookup_transform('cf1/odom', source_frame,rospy.Time(0), rospy.Duration(1.0))
        except:
            #trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Failure of lookup transfrom from cf1/odom to %s' % source_frame)
        
        if trans!= None:

            #print (m)
            self.x_odom = trans.transform.translation.x
            self.y_odom = trans.transform.translation.y
            self.z_odom = trans.transform.translation.z
            
            self.x_quat_odom = round(trans.transform.rotation.x, 1)
            self.y_quat_odom = round(trans.transform.rotation.y, 1)
            self.z_quat_odom = round(trans.transform.rotation.z, 1)
            self.w_quat_odom = round(trans.transform.rotation.w, 1)      

            print(self.x_quat_odom)
            print(self.y_quat_odom)
            print(self.z_quat_odom)
            print(self.w_quat_odom)

            (self.roll_odom,
            self.pitch_odom,
            self.yaw_odom,) = euler_from_quaternion([self.x_quat_odom, self.y_quat_odom, self.z_quat_odom, self.w_quat_odom],axes='szxy') #output in radians
            
            print(self.roll_odom)
            print(self.pitch_odom)
            print(self.yaw_odom)
            
            self.roll_odom= round(self.roll_odom,3)
            self.pitch_odom= round(self.pitch_odom,3)
            self.yaw_odom= round(self.yaw_odom,3)

            # Converting into degrees to subtract from map orientation angles which are in euler i.e in degrees
            self.roll_odom = math.degrees(self.roll_odom)
            self.pitch_odom = math.degrees(self.pitch_odom)
            self.yaw_odom = math.degrees(self.yaw_odom)

            print(self.roll_odom)
            print(self.pitch_odom)
            print(self.yaw_odom)


            self.marker_map(marker)
            self.diff()
            transform = self.odom_to_map()
            print("Done")
            print(transform)
            self.br.sendTransform(transform)
            rospy.sleep(3)

        else:
            rospy.loginfo("No transform available")

        return transform


    def main(self, argv=sys.argv):
        
        rate = rospy.Rate(10)  # Hz

        while not rospy.is_shutdown():

            # Let ROS filter through the arguments
            args = rospy.myargv(argv=argv)

            if args!= None:
            # Load input world JSON
                with open(args[1], 'rb') as f:
                    self.world = json.load(f)
                
            else:  
            # Load default world JSON
                with open('~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone2.world.json', 'rb') as f:
                    self.world = json.load(f)

            #transform = self.marker_odom()
        
            for m in self.world['markers']:
            #    transforms = [transform_from_marker(m)]
                self.marker_odom(m)
                #if transform!= None:

                    # Publish these transforms statically forever

            rate.sleep()

if __name__ == '__main__':


    rospy.init_node('map_to_odom', anonymous=True)
    rospy.loginfo("Successful initilization of node map_to_odom")

    odom_to_map = Localization()
    odom_to_map.main()
    #rospy.spin()