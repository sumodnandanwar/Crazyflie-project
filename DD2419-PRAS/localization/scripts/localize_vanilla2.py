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
import PyKDL
import tf_conversions.posemath as pm

from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover

from std_msgs.msg import Empty
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker

#from binary_map_4n import localization_algo


class Localization:

    def __init__(self):
        

        # Initilize tf2 broadcaster and transform message
        self.br = tf2_ros.StaticTransformBroadcaster()
        # self.br = tf2_ros.TransformBroadcaster()

        # self.br = tf2_ros.StaticTransformBroadcaster()
        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # rospy.sleep(2)

    def marker_map(self, m):
        
        x_map, y_map, z_map = m['pose']['position']

        roll_map, pitch_map, yaw_map = m['pose']['orientation']       
        self.goal_map = PoseStamped()
        self.goal_map.header.stamp = rospy.Time.now()
        self.goal_map.header.frame_id = "map"
        self.goal_map.pose.position = (x_map, y_map, z_map)
        (self.goal_map.pose.orientation.x,
        self.goal_map.pose.orientation.y,
        self.goal_map.pose.orientation.z,
        self.goal_map.pose.orientation.w) = quaternion_from_euler(math.radians(roll_map),
                                                                math.radians(pitch_map),
                                                                math.radians(yaw_map),'rzxy')         # rzxy


        self.map_pykdl = PyKDL.Frame(PyKDL.Rotation.RPY(round(math.radians(roll_map),1),round(math.radians(pitch_map),1),round(math.radians(yaw_map),1)), PyKDL.Vector(x_map,y_map,z_map))

    def diff(self):
        
        self.map_odom_diff = self.odom_pykdl * self.map_pykdl.Inverse()     # This is the transformation from pose2 i.e.odom to pose1 i.e. map or "pose1 - pose2"
        
        # print(self.map_odom_diff)


    def odom_to_map(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'cf1/odom'
        t.child_frame_id = 'map'
        t.transform.translation.x = self.map_odom_diff[(0, 3)]
        t.transform.translation.y = self.map_odom_diff[(1, 3)]
        t.transform.translation.z = self.map_odom_diff[(2, 3)]
        
        # [self.roll_diff, self.pitch_diff, self.yaw_diff] = self.map_odom_diff.GetRPY()
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = self.map_odom_diff.M.GetQuaternion()
        # (t.transform.rotation.x,
        # t.transform.rotation.y,
        # t.transform.rotation.z,
        # t.transform.rotation.w) = quaternion_from_euler(math.radians(self.roll_diff),
        #                                              math.radians(self.pitch_diff),
        #                                              math.radians(self.yaw_diff))       # ,'rzxy'

        # print(t)
        # self.br2.sendTransform(t)
        # rospy.sleep(3600)

        return t

    def odom_to_map_static(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'cf1/odom'
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        
        (t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w) = (0,0,0,1)
        # print(t)

        # self.br.sendTransform(t)
        # rospy.sleep(1)
        return t



    def marker_odom(self, marker):
        
        trans = None
        transform = TransformStamped()
        trans_static = False
        trans_detected = False

        source_frame = "aruco/detected" + str(marker['id'])

        try:
            trans = self.tfBuffer.lookup_transform('cf1/odom', source_frame,rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Success of lookup transfrom from %s to cf1/odom' % source_frame)

        except:
            #trans = self.tfBuffer.lookup_transform(self.map_frame, self.est_veh_pose_frame, rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Failure of lookup transfrom from %s to cf1/odom' % source_frame)
        
        if trans!= None:
            # (position_trans, quartenion_trans) = (trans.transform.translation, trans.transform.rotation)
            tf_trans = ((trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z), (trans.transform.rotation.x, 
                        trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
            self.odom_pykdl = pm.fromTf(tf_trans)

            #print (m)
            self.x_odom = trans.transform.translation.x
            self.y_odom = trans.transform.translation.y
            self.z_odom = trans.transform.translation.z
            
            # print("Translation odom is")
            # print(self.x_odom)
            # print(self.y_odom)
            # print(self.z_odom)

            self.x_quat_odom = round(trans.transform.rotation.x, 1)
            self.y_quat_odom = round(trans.transform.rotation.y, 1)
            self.z_quat_odom = round(trans.transform.rotation.z, 1)
            self.w_quat_odom = round(trans.transform.rotation.w, 1)      

            # print(self.x_quat_odom)
            # print(self.y_quat_odom)
            # print(self.z_quat_odom)
            # print(self.w_quat_odom)


            self.marker_map(marker)
            self.diff()
            transform = self.odom_to_map()
            trans_detected = True
            rospy.loginfo("Done")
            # self.br.sendTransform(transform)
            # rospy.sleep(1)

        else:
            
            # if self.bool_static is False:
            rospy.loginfo("No transform available")
            # transform = self.odom_to_map_static()
            # trans_static = True
            # rospy.loginfo("Published static transform")
            # print(transform)
            # self.br.sendTransform(transform)
            # rospy.sleep(1)
            #     self.bool_static =True

            #rospy.sleep(3)

        return trans_static, trans_detected , transform


    def main(self, argv=sys.argv, transform_bool = False):
        
        transform = TransformStamped()
        trans_static = False
        trans_detected = False
        while transform_bool is False:

            # Let ROS filter through the arguments
            args = rospy.myargv(argv=argv)

            if args!= None:
            # Load input world JSON
                with open(args[1], 'rb') as f:
                    self.world = json.load(f)
                
            else:  
            # Load default world JSON
                with open('~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json', 'rb') as f:
                    self.world = json.load(f)

            #transform = self.marker_odom()
        
            for m in self.world['markers']:
            #    transforms = [transform_from_marker(m)]
                trans_static, trans_detected , transform = self.marker_odom(m)
                
                if trans_detected is True:
                    rospy.loginfo("Transform available")

                    self.br.sendTransform(transform)
                    rospy.sleep(3)

                    # transform_bool = True       # Set to True to get the first transform and keep on publishing that
                    transform_bool = True

                    break
                elif trans_static is True:
                    rospy.loginfo("No transform available")
                    # self.br.sendTransform(transform)
                    # self.odom_to_map_static()
                    rospy.sleep(1)
                    # rospy.loginfo("Published static transform")

                    transform_bool = False
        
        return transform
        # rospy.spin()


        # rate.sleep()

if __name__ == '__main__':

    transform = TransformStamped()
    rospy.init_node('map_to_odom', anonymous=True)
    rospy.loginfo("Successful initilization of node map_to_odom")

    odom_to_map = Localization()

    transform = odom_to_map.main()
    
    # rate = rospy.Rate(10)
    # br2 = tf2_ros.StaticTransformBroadcaster()
    # while not rospy.is_shutdown():
    # transform.header.stamp = rospy.Time.now()
    # transform.header.frame_id = 'map'
    # transform.child_frame_id = 'cf1/odom'

    # br2.sendTransform(transform)
    print(transform)
    rospy.spin()
        # rate.sleep