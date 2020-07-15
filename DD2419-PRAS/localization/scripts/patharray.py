#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import Waypoint_goals


patharray = []
x = 0
y = 0




def odom_cb(data):
    pose = [x,y]
    pose[0] = data.pose.position.x
    pose[1] = data.pose.position.y
    patharray.append(pose)
    print Waypoint_goals.main()
    # path_pub.publish(patharray)


rospy.init_node('Path_array')


odom_sub = rospy.Subscriber('/cf1/pose', PoseStamped, odom_cb)
# path_pub = rospy.Publisher('/path', Path, queue_size=10)

if __name__ == '__main__':
    rospy.spin()