#!/usr/bin/env python
import json
import sys
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Empty
from crazyflie_driver.msg import Position
from crazyflie_driver.msg import Hover
from tf2_msgs.msg import TFMessage
from aruco_msgs.msg import MarkerArray
from aruco_msgs.msg import Marker

import geometry_msgs.msg

pose = None

marker_id = None
# pos = Position()
position = Position()

currentPose = None

def updateCurrentPose(msg):
    global currentPose
    currentPose = msg.pose

def pose_callback(data):
    global pose,marker_id
    marker_pos = Marker()
    pose_camera = PoseStamped()
    for m in data.markers:
        if m.id != 0:
            marker_pos = m
    
    marker_id = marker_pos.id
    
    pose_camera = PoseStamped()
    pose_camera.header.frame_id = "cf1/camera_link"
    pose_camera.pose.position= marker_pos.pose.pose.position
    pose_camera.pose.orientation = marker_pos.pose.pose.orientation

    if not tf_buf.can_transform("cf1/base_link", 'cf1/camera_link', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % pose_camera.header.frame_id)
        return
    

    pose_base = tf_buf.transform(pose_camera,'cf1/base_link')
    # # pose_base = tf_buf.lookup_transform("cf1/base_link","cf1/camera_link",rospy.Time(0.0))
    
    # rospy.loginfo("pose_base")
    # rospy.loginfo(pose_base)

    if not tf_buf.can_transform("cf1/odom", 'cf1/base_link', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % pose_base.header.frame_id)
        return

    pose_odom = tf_buf.transform(pose_base,'cf1/odom')

    pose = pose_odom


    broadcaster1 = tf2_ros.StaticTransformBroadcaster()
    t1 = geometry_msgs.msg.TransformStamped()

    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time(0.0)
    t1.header.frame_id = "map"
    t1.child_frame_id = "aruco/detect"+str(marker_id)

    t1.transform.translation = pose_odom.pose.position
    t1.transform.rotation = pose_odom.pose.orientation

    broadcaster1.sendTransform(t1)


def publish_cmd(cmd):
    global pose,position
    pub_cmd.publish(cmd)
    if pose:
        position.z = pose.pose.position.z
    pose = None
        
# get cmd from poseStamped
def getCmd():
    global pose,currentPose
    while currentPose == None or pose==None:
        continue
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "cf1/odom"

    cmd.x = pose.pose.position.x
    cmd.y = pose.pose.position.y
    cmd.z = pose.pose.position.z
    # cmd.z = 0.3

    _,_,yaw = euler_from_quaternion((pose.pose.orientation.x,
                                    pose.pose.orientation.y,
                                    pose.pose.orientation.z,
                                    pose.pose.orientation.w))
    cmd.yaw = math.degrees(yaw)

    return cmd

# def getCmdFromCurrentPose():
#     global currentPose
#     while currentPose == None:
#         continue
#     cmd = Position()

#     cmd.header.stamp = rospy.Time.now()
#     cmd.header.frame_id = "cf1/odom"

#     cmd.x = currentPose.position.x
#     cmd.y = currentPose.position.y
#     cmd.z = currentPose.position.z
    

#     _,_,yaw = euler_from_quaternion((currentPose.orientation.x,
#                                     currentPose.orientation.y,
#                                     currentPose.orientation.z,
#                                     currentPose.orientation.w))
#     cmd.yaw = math.degrees(yaw)

#     return cmd

rospy.init_node("follow_marker",anonymous=True)
rospy.Subscriber("/aruco/markers",MarkerArray,pose_callback)

pose_sub = rospy.Subscriber("/cf1/pose",PoseStamped,updateCurrentPose)

pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

tf_buf = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buf)

def main():
    # global pub_cmd
    rate = rospy.Rate(20)
    # set the initial position height to 0.3
    position = Position()
    position.x = 0.15
    position.y = 0.15
    position.z = 0.35
    
    while not rospy.is_shutdown():
        # rospy.loginfo(pose)
        # rospy.loginfo(currentPose)
        if pose:
            cmd = getCmd()   
            cmd.x -= 0.4
            publish_cmd(cmd)
            rospy.loginfo(cmd)
        # elif currentPose:
        #     cmd1 = getCmdFromCurrentPose()
        #     cmd1.z = 0.3
        #     pub_cmd.publish(cmd1)
        #     rospy.loginfo(cmd1)
        else:
            pub_cmd.publish(position)
            rospy.loginfo(position)
        rate.sleep()

if __name__ == "__main__":
    main()


            
    
