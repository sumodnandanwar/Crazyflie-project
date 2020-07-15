#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3
from os.path import expanduser

def transform_from_object(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'object/map' + str(m['sign'])
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t

def main():

    # Open JSON file
    path = expanduser('~')
    path += "/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/milestone3.world.json"
    with open(path, "rb") as f:
        world = json.load(f)

    # Create a transform for each marker
    transforms = [transform_from_object(m) for m in world['roadsigns']]

    # Publish these transforms statically forever
    rospy.init_node('displaymapobjects')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform(transforms)
    rospy.spin()

if __name__ == "__main__":
    main()