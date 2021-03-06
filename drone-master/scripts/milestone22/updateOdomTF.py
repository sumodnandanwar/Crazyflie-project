#!/usr/bin/env python

from os.path import expanduser
import rospy
from aruco_msgs.msg import MarkerArray, Marker
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseWithCovariance, PoseStamped, TransformStamped
from crazyflie_driver.msg import Position
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Empty
import json


driftMarker = None
odomTrans = TransformStamped()


def update_callback(marker):
    global driftMarker
    if tf_buf.can_transform(marker.header.frame_id, "map", rospy.Time(0)):
        rospy.loginfo('hejhej')
        driftMarker = tf_buf.transform(marker, "map", rospy.Duration(0.1))


def updateOdom(jsonMarker):
    global driftMarker, odomTrans

    xdiff = jsonMarker['pose']['position'][0]-driftMarker.pose.position.x
    ydiff = jsonMarker['pose']['position'][1]-driftMarker.pose.position.y
    zdiff = jsonMarker['pose']['position'][2]-driftMarker.pose.position.z

    odomTrans.header.stamp = rospy.Time.now()
    odomTrans.transform.translation.x = xdiff
    odomTrans.transform.translation.y = ydiff
    odomTrans.transform.translation.z = zdiff
    odomTrans.transform.rotation.w = 1

    rospy.loginfo('xdiff = ')
    rospy.loginfo(xdiff)
    rospy.loginfo('ydiff = ')
    rospy.loginfo(ydiff)
    rospy.loginfo('zdiff = ')
    rospy.loginfo(zdiff)


rospy.init_node('update_odom_tf')
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
sub_tf = rospy.Subscriber('update_ready_odom_map',
                          PoseStamped, update_callback)


def main():

    rate = rospy.Rate(10)
    odomTrans.header.frame_id = 'map'
    odomTrans.child_frame_id = 'cf1/odom'
    odomTrans.transform.rotation.w = 1

    rospy.loginfo(odomTrans)

    # Open JSON file
    path = expanduser('~')
    path += "/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json"
    with open(path, "rb") as f:
        world = json.load(f)

    jsonMarkers = [m for m in world['markers']]

    i = 0
    while not driftMarker and not rospy.is_shutdown():

        odomTrans.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(odomTrans)
        print(i)
        i += 1
        rate.sleep()

    while not rospy.is_shutdown():

        updateOdom(jsonMarkers[0])
        broadcaster.sendTransform(odomTrans)
        rate.sleep()


if __name__ == '__main__':
    main()
