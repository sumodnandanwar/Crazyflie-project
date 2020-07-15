#!/usr/bin/env python
import math
import numpy as np
import rospy
from os.path import expanduser
import csv
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty, Int16, String

# def Dict_goal():
#     global dictgoal
#     dictgoal = []
#     pathcsv = expanduser('~')
#     pathcsv += '/dd2419_ws/src/DD2419-PRAS/localization/scripts/Pathcsv'
#     with open(pathcsv, 'rb') as csv_file:
#         csv_reader = csv.reader(csv_file)
#         next(csv_reader)
#         for line in csv_reader:
#             line[0] = float(line[0])
#             line[1] = float(line[1])
#             dictgoal.append(line)
#     # print dictgoal[0][1]
#     return dictgoal

# def goalpub(cmdx,cmdy):
#     global odom_cmd, cmd

#     # rate = rospy.Rate(10)
#     goal = PoseStamped()
#     goal.header.stamp = rospy.Time.now()
#     goal.header.frame_id = "map"
#     goal.pose.position.x = cmdx
#     goal.pose.position.y = cmdy
#     goal.pose.position.z = 0.4
#     if not tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
#         rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
#         return

#     goal_odom = tf_buf.transform(goal, 'cf1/odom')

#     if goal_odom:
#         cmd = Position()
#         cmd.header.stamp = rospy.Time.now()
#         cmd.header.frame_id = "cf1/odom"
#         cmd.x = goal_odom.pose.position.x + 0.5
#         cmd.y = goal_odom.pose.position.y + 0.5
#         cmd.z = goal_odom.pose.position.z
#         cmd.yaw = 180
#         odom_cmd = cmd
#         # print odom_cmd
#         print odom_cmd.x, odom_cmd.y

msg = 0

def sub_callback(cf1_pose):
    print cf1_pose
    msg = cf1_pose.data
    print msg

rospy.init_node('Takeoff_n_Scout')
sub_pose = rospy.Subscriber('Object_Trig', Int16, sub_callback)
# tf_buf = tf2_ros.Buffer()
# tf_lstn = tf2_ros.TransformListener(tf_buf)

# def main():
#     Dict_goal()
#     print dictgoal
#     for e in range(len(dictgoal)):
#         xcmd = dictgoal[e][0]
#         ycmd = dictgoal[e][1]
#         goalpub(xcmd,ycmd)
         

if __name__ == '__main__':
    # main()
    rospy.spin()