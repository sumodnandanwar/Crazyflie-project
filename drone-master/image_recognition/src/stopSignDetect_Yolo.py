#!/usr/bin/env python
from __future__ import print_function

from os.path import expanduser

import numpy as np
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs


f = 231
trueHeight = 0.2


def box_callback(msg):
    global f, trueHeight

    # TF transform
    object_camera = PoseStamped()
    object_camera.header.frame_id = 'cf1/camera_link'
    object_camera.header.stamp = rospy.Time.now()

    for box in msg.bounding_boxes:
        if box.Class == 'stop sign':  # and box.probability > 0.5:
            rospy.loginfo([
                'Stop sign detected with probability: ', box.probability])
            height = box.ymax-box.ymin
            width = box.xmax-box.xmin
            x = box.xmin+width/2 - 320
            y = box.ymin+height/2 - 340
            Z = (f*trueHeight)/height
            X = (y*Z)/f
            Y = (x*Z)/f
            object_camera.pose.position.x = X
            object_camera.pose.position.y = Y
            object_camera.pose.position.z = Z

            if not tf_buf.can_transform(object_camera.header.frame_id, 'map', rospy.Time(0)):
                rospy.logwarn_throttle(
                    5.0, 'No transform from %s to map' % object_camera.header.frame_id)
                return

            objectMap = tf_buf.transform(
                object_camera, 'map', rospy.Duration(0.5))

            trans = TransformStamped()
            trans.header.frame_id = 'map'
            trans.child_frame_id = 'stopsign'
            trans.header.stamp = rospy.Time.now()
            trans.transform.translation = objectMap.pose.position
            trans.transform.rotation.w = 1
            broadcaster.sendTransform(trans)


rospy.init_node('stopSignDetect')
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)
broadcaster = tf2_ros.TransformBroadcaster()
bounds_sub = rospy.Subscriber(
    '/darknet_ros/bounding_boxes', BoundingBoxes, box_callback)

if __name__ == "__main__":

    rospy.spin()
