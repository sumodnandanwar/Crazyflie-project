#!/usr/bin/env python

import sys
import math
import json

import rospy
import tf2_ros 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped, Vector3

###


class MatPlot:
    def __init__(self, node_name, world_file=None):
        # Let ROS filter through the arguments
        args = rospy.myargv(argv=sys.argv)
    
        # Load world JSON
        if len(args) > 1:
            world_file = args[1]
            
        with open(world_file, 'rb') as f:
            world = json.load(f)
    
        # Create a transform for each marker and trafic sign
        transforms = []
        
        try:
            transforms += [transform_from_marker(m) for m in world['markers']]
        except KeyError:
            pass
        
        try:
            transforms += [transform_from_roadsign(r) for r in world['roadsigns']]
        except KeyError:
            pass
    
        # Publish these transforms statically forever
        rospy.init_node(node_name)
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        broadcaster.sendTransform(transforms)
        rospy.spin()


def transform_from_marker(m):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'aruco/marker' + str(m['id'])
    t.transform.translation = Vector3(*m['pose']['position'])
    roll, pitch, yaw = m['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t


def transform_from_roadsign(r):
    t = TransformStamped()
    t.header.frame_id = 'map'
    t.child_frame_id = 'roadsign/sign_' + str(r['sign'])
    t.transform.translation = Vector3(*r['pose']['position'])
    roll, pitch, yaw = r['pose']['orientation']
    (t.transform.rotation.x,
     t.transform.rotation.y,
     t.transform.rotation.z,
     t.transform.rotation.w) = quaternion_from_euler(math.radians(roll),
                                                     math.radians(pitch),
                                                     math.radians(yaw))
    return t

###


if __name__ == "__main__":
    mat_plot = MatPlot('MatPlot', 
                       '/home/ff/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/aruco.world.json')