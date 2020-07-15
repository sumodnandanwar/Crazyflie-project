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

goal = None

marker_id = None
# pos = Position()
position = Position()

currentPose = None

def updateCurrentPose(msg):
    global currentPose
    currentPose = msg.pose

def goal_callback(data):
    global goal,marker_id
    marker_pos = Marker()
    goal_camera = PoseStamped()
    for m in data.markers:
        if m.id != 0:
            marker_pos = m
    
    marker_id = marker_pos.id
    
    goal_camera = PoseStamped()
    goal_camera.header.frame_id = "cf1/camera_link"
    goal_camera.pose.position= marker_pos.pose.pose.position
    goal_camera.pose.orientation = marker_pos.pose.pose.orientation

    if not tf_buf.can_transform("cf1/base_link", 'cf1/camera_link', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/base_link' % goal_camera.header.frame_id)
        return
    

    goal_base = tf_buf.transform(goal_camera,'cf1/base_link')
    # # goal_base = tf_buf.lookup_transform("cf1/base_link","cf1/camera_link",rospy.Time(0.0))
    
    # rospy.loginfo("goal_base")
    # rospy.loginfo(goal_base)

    if not tf_buf.can_transform("cf1/odom", 'cf1/base_link', rospy.Time(0.0)):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal_base.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal_base,'cf1/odom')

    goal = goal_odom


    broadcaster1 = tf2_ros.StaticTransformBroadcaster()
    t1 = geometry_msgs.msg.TransformStamped()

    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time(0.0)
    t1.header.frame_id = "map"
    t1.child_frame_id = "aruco/detect"+str(marker_id)

    t1.transform.translation = goal_odom.pose.position
    t1.transform.rotation = goal_odom.pose.orientation

    broadcaster1.sendTransform(t1)


def publish_cmd(cmd):
    global goal,position
    pub_cmd.publish(cmd)
    if goal:
        position.z = goal.pose.position.z
    goal = None
        
# get cmd from poseStamped
def getCmd():
    global goal,currentPose
    while currentPose == None or goal==None:
        continue
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = "cf1/odom"

    cmd.x = goal.pose.position.x
    cmd.y = goal.pose.position.y
    cmd.z = goal.pose.position.z
    # cmd.z = 0.3

    _,_,yaw = euler_from_quaternion((goal.pose.orientation.x,
                                    goal.pose.orientation.y,
                                    goal.pose.orientation.z,
                                    goal.pose.orientation.w))
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
rospy.Subscriber("/aruco/markers",MarkerArray,goal_callback)

pose_sub = rospy.Subscriber("/cf1/pose",PoseStamped,updateCurrentPose)

pub_cmd = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)

tf_buf = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buf)

def main():
    # global pub_cmd
    rate = rospy.Rate(20)
    # set the initial position height to 0.3
    position = Position()
    position.x = 0.5
    position.y = 0.5
    position.z = 0.3
    
    while not rospy.is_shutdown():
        # rospy.loginfo(goal)
        # rospy.loginfo(currentPose)
        if goal:
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


            
    
