#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 23:15:51 2020

@author: Fredrik Forsberg
"""

import rospy
import time
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped

from pose_info import PoseInfoClass

###


class FlyDrone:
    def __init__(self, node_name, cmd_publishing_topic, fly_subscription_topic, reached_distance=0.1, 
                 reached_angle=np.pi/36., reached_stabilize=1):
        # Desired travel positions
        self.targets = []
        
        # Distance at which the target is considered reached
        self.reached_distance = reached_distance  # m
        # Difference in yaw when the angle is considered reached
        self.reached_angle = reached_angle  # radians
        # Time for the drone to stabilize
        self.reached_stabilize = reached_stabilize  # s
        # Time at which the target was reached
        self.reached_timestamp = 0
        
        # Create node
        rospy.init_node(node_name)
        
        # TF buffer
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lstn  = tf2_ros.TransformListener(self.tf_buf)
        
        # Create publisher
        self.cmd_publisher = rospy.Publisher(cmd_publishing_topic, Position, queue_size=2)
        # Create subscriber to incomming commands
        rospy.Subscriber(fly_subscription_topic, PoseStamped, self.add_target)
        # Create listener for pose
        self.pose_info = PoseInfoClass()
    
        
    def add_target_coords(self, coords):
        # Translates the coordinates to a PoseStamped
        # coords = [x, y, z, yaw]
        map_pose = PoseStamped()
        
        if len(coords) > 4:
            map_pose.header.frame_id = coords[4]
        else:
            map_pose.header.frame_id = 'map'
        
        map_pose.pose.position.x = coords[0]
        map_pose.pose.position.y = coords[1]
        map_pose.pose.position.z = coords[2]
        
        yaw = np.radians(coords[3])
        
        (map_pose.pose.orientation.x,
         map_pose.pose.orientation.y,
         map_pose.pose.orientation.z,
         map_pose.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
        
        self.add_target(map_pose)
    
    
    def add_target(self, pose):
        self.targets.append(pose)
    
    
    def to_odom_pose(self, pose):
        if pose.header.frame_id == 'cf1/odom':
            return pose
        
        pose.header.stamp = rospy.Time(0.0)
        
        if not self.tf_buf.can_transform(pose.header.frame_id, 'cf1/odom', rospy.Time(0.0)):
            rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % pose.header.frame_id)
            return None
    
        odom_pose = self.tf_buf.transform(pose, 'cf1/odom')
        odom_transform = self.tf_buf.lookup_transform(pose.header.frame_id, 'cf1/odom', rospy.Time(0.0))
        odom_pose.header.stamp = odom_transform.header.stamp
        
        return odom_pose
    
    
    def pose_to_cmd(self, odom_pose):
        
        if odom_pose is None:
            return None
    
        cmd = Position()
    
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = odom_pose.header.frame_id
    
        cmd.x = odom_pose.pose.position.x
        cmd.y = odom_pose.pose.position.y
        cmd.z = odom_pose.pose.position.z
    
        roll, pitch, yaw = euler_from_quaternion((odom_pose.pose.orientation.x,
                                                  odom_pose.pose.orientation.y,
                                                  odom_pose.pose.orientation.z,
                                                  odom_pose.pose.orientation.w))
    
        cmd.yaw = np.degrees(yaw)
        
        return cmd
    
    
    def get_target(self):
        if len(self.targets) == 0:
            return None
        else:
            new_target = self.targets.pop(0)
            
            # The below code was commented out since rospy.Time.to_sec(rospy.Time.now()) is seconds since roslaunch
            #     and not seconds since 1970 as with time.time()
            # 
            # If the header stamp of the new target is between 0 and 30, then it's interpreted as a message to
            #     change the stabilisation time 
            # if new_target.header.stamp > rospy.Time.from_sec(0) and new_target.header.stamp < rospy.Time.from_sec(30):
            #     self.reached_stabilize = new_target.header.stamp
                
            new_target.header.stamp = rospy.Time.now()
            
            if new_target.header.frame_id != 'cf1/odom':
                return new_target
            
            else:
                # odom frame => Interpret the target to be relative to the current pose
                p_x, p_y, p_z = self.pose_info.get_position()
                _, _, p_yaw = self.pose_info.get_angles()
                
                new_target.pose.position.x += p_x
                new_target.pose.position.y += p_y
                new_target.pose.position.z += p_z
                
                roll, pitch, yaw = euler_from_quaternion((new_target.pose.orientation.x,
                                                          new_target.pose.orientation.y,
                                                          new_target.pose.orientation.z,
                                                          new_target.pose.orientation.w))
                
                (new_target.pose.orientation.x,
                 new_target.pose.orientation.y,
                 new_target.pose.orientation.z,
                 new_target.pose.orientation.w) = quaternion_from_euler(roll, pitch, yaw + p_yaw)
                
                return new_target
        
        
    def get_distance_to_target(self, odom_target):
        # Both odom_target and self.pose_info.pose are in the odom frame
        if odom_target is None:
            # If odom_target is None we need to get a new target => We say that the distance is 0
            return 0
        
        pose_coords = np.asarray([self.pose_info.pose.pose.position.x, self.pose_info.pose.pose.position.y, 
                                  self.pose_info.pose.pose.position.z],
                                 dtype=np.float64)
        target_coords = np.asarray([odom_target.pose.position.x, odom_target.pose.position.y, 
                                    odom_target.pose.position.z], dtype=np.float64)
        
        return np.linalg.norm(pose_coords - target_coords)
    
    
    def get_angle_from_target_angle(self, odom_target):
        _, _, t_yaw = euler_from_quaternion((odom_target.pose.orientation.x,
                                             odom_target.pose.orientation.y,
                                             odom_target.pose.orientation.z,
                                             odom_target.pose.orientation.w))
        
        _, _, p_yaw = self.pose_info.get_angles()
        
        return abs((t_yaw - p_yaw) % (2*np.pi))
    
    
    def map_connection_lost(self):
        # TODO Scan the surroundings for markers/signs?
        pass
    
    #
    
    
    def run(self):
        rate = rospy.Rate(10)  # Hz
        
        target = self.get_target()
        odom_target = None
        stabilizing = False
        
        while not rospy.is_shutdown():
            
            if target is not None:
                updated_odom_target = self.to_odom_pose(target)
            
                if updated_odom_target is not None:
                    odom_target = updated_odom_target
                    print(odom_target)  # TODO Remove
            
            if self.get_distance_to_target(odom_target) <= self.reached_distance:
                # Within an acceptable proximity to the target
                
                if ((time.time() - self.reached_timestamp > self.reached_stabilize and stabilizing and 
                     self.get_angle_from_target_angle(odom_target) <= self.reached_angle) 
                    or odom_target is None):  # TODO
                    # Stabilizing done. Get new target.
                    stabilizing = False
                    
                    new_target = self.get_target()
                    
                    if new_target is not None:
                        target = new_target
                    else:
                        self.map_connection_lost()
                
                elif time.time() - self.reached_timestamp > self.reached_stabilize and not stabilizing:
                    # Just reached the target. Set timestamp.
                    stabilizing = True
                    self.reached_timestamp = time.time()
                    
                    
            if odom_target is not None:
                # Transform to map
                cmd = self.pose_to_cmd(odom_target)
                if cmd is not None:
                    self.cmd_publisher.publish(cmd)
                
                
            rate.sleep()
        
        # rospy.spin() shouldn't be needed since it sleeps until rospy.is_shutdown() returns True
        # Keeping it to be on the safe side
        rospy.spin()

###


if __name__ == '__main__':
    fly_drone = FlyDrone('FlyDrone', '/cf1/cmd_position', '/fly_pose', 
                         reached_distance=0.1, reached_angle=np.pi/36., reached_stabilize=1)
    
    for item in [[0.5, 0.5, 0.4, 0, 'cf1/odom'], [0, 0, 0.4, 0]]:  # x, y, z, yaw, (frame)
        fly_drone.add_target_coords(item)
        # TODO Will the initial liftoff be done in PathPlanning?
    
    fly_drone.run()
