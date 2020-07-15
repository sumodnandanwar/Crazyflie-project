#!/usr/bin/env python

import sys
import math
import rospy
import json
import string
import tf2_ros
import tf2_geometry_msgs
from numpy import mean , average
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



class Localization:

    def __init__(self):
        
        # Initilize tf2 broadcaster and transform message
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.t = TransformStamped()
        # Initialize listener for estimated pose of vehicle in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # rospy.sleep(2)
        self.initialize = 0
        self.t1 = TransformStamped()

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
        
        self.odom_map_diff = self.map_pykdl * self.odom_pykdl.Inverse()     # This is the transformation from pose2 i.e.map to pose1 i.e. odom or "pose1 - pose2"
        
        print(self.odom_map_diff)


    def odom_to_map(self):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = 'map'
        t.child_frame_id = 'cf1/odom'
        t.transform.translation.x = self.odom_map_diff[(0, 3)]
        t.transform.translation.y = self.odom_map_diff[(1, 3)]
        t.transform.translation.z = self.odom_map_diff[(2, 3)]
        
        (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w) = self.odom_map_diff.M.GetQuaternion()

        print('t'),t
        return t

    #This function returns the weighted average of the input list
    def moving_ave(self, pos_list):
        moving_ave_list = pos_list
        avg = 0.0
        weight = [0.4,0.3,0.1,0.2]
        avg = average(moving_ave_list, weights=weight)
        return avg

    #This function returns creates a new transformStamped msg using the average values
    def tavg_calc(self, size):
        xa, ya, za, x_q, y_q, z_q, w_q = [], [], [], [], [], [], []
        twa = self.odom_to_map()
        tavg = TransformStamped()
        tavg.header.frame_id = 'map'
        tavg.child_frame_id = 'cf1/odom'
        tavg.header.stamp = rospy.Time.now()

        if twa is not None:

            for i in range(size):
                xa.append(twa.transform.translation.x)
                ya.append(twa.transform.translation.y)
                za.append(twa.transform.translation.z)

                x_q.append(twa.transform.rotation.x)
                y_q.append(twa.transform.rotation.y)
                z_q.append(twa.transform.rotation.z)
                w_q.append(twa.transform.rotation.w)
            

            x_pos = self.moving_ave(xa)
            y_pos = self.moving_ave(ya)
            z_pos = self.moving_ave(za)


            
            x_q_pos = twa.transform.rotation.x
            y_q_pos = twa.transform.rotation.y
            z_q_pos = twa.transform.rotation.z
            w_q_pos = twa.transform.rotation.w
            
            tavg.transform.translation.x = x_pos
            tavg.transform.translation.y = y_pos
            tavg.transform.translation.z = z_pos
                
            tavg.transform.rotation.x = x_q_pos
            tavg.transform.rotation.y = y_q_pos
            tavg.transform.rotation.z = z_q_pos
            tavg.transform.rotation.w = w_q_pos

            print ('tavg'), tavg
            return tavg

    def first_tf(self):

        if self.initialize < 1:
            self.t1 = self.odom_to_map()
            self.initialize = 1
        # print't1' , self.t1
        return self.t1

    def dist(self, myposex, myposey, myposez, goalx, goaly, goalz):
        dis = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2+(myposez-goalz)**2)
        print dis
        return dis

    def marker_odom(self, marker):
        
        trans = None
        # transform = TransformStamped()
        trans_detected = False

        source_frame = "aruco/detected" + str(marker['id'])

        try:
            trans = self.tfBuffer.lookup_transform('cf1/odom', source_frame,rospy.Time(0), rospy.Duration(1.0))
            rospy.loginfo('Success of lookup transfrom from %s to cf1/odom' % source_frame)

        except:
            rospy.loginfo('Failure of lookup transfrom from %s to cf1/odom' % source_frame)
        
        if trans!= None:

            tf_trans = ((trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z), (trans.transform.rotation.x, 
                        trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
            self.odom_pykdl = pm.fromTf(tf_trans)

            self.marker_map(marker)
            self.diff()
            trans_detected = True

        else:
            
            rospy.loginfo("No transform available")

        # return trans_detected , transform
        return trans_detected


    def main(self, argv=sys.argv, transform_main = TransformStamped(), transform_bool = False):
        
        transform = transform_main
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
        
            for m in self.world['markers']:

                trans_detected = self.marker_odom(m)
                
                if trans_detected is True:
                    rospy.loginfo("Transform available and broadcasted")
                    transform = self.odom_to_map()
                    transform_bool = True
                    break
                else:
                    rospy.loginfo("No transform available")
                    transform_bool = False
        
        return transform

if __name__ == '__main__':
    rospy.init_node('map_to_odom', anonymous=True)
    rospy.loginfo("Successful initilization of node map_to_odom")

    rate = rospy.Rate(5)        
    br = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()
    odom_to_map = Localization()

    while not rospy.is_shutdown():

        transform = odom_to_map.main(transform_main=transform)
        transform.header.stamp = rospy.Time.now()
        br.sendTransform(transform)
        rospy.sleep(0.1)

    rate.sleep()
