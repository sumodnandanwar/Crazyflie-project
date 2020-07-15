#!/usr/bin/env python

import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position

state = 0
angle = 0
# Current goal (global state)
goal = None

def takeoff_callback(msg):
    takeoff_cmd = Position()

    takeoff_cmd.header.stamp = rospy.Time.now()
    takeoff_cmd.header.frame_id = 'cf1/odom'

    takeoff_cmd.x = 0
    takeoff_cmd.y = 0
    takeoff_cmd.z = 0.4
    pub_cmd.publish(takeoff_cmd)

def Rotate_callback(goal):
    goal.header.stamp = rospy.Time.now()

    if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
        rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    global state, angle, lap, N, origin
    
    Rotate_cmd = Position()
    Rotate_cmd.header.stamp = rospy.Time.now()   
    Rotate_cmd.header.frame_id = 'cf1/odom'
    angle += 5 
    Rotate_cmd.x = 0
    Rotate_cmd.y = 0
    Rotate_cmd.z = 0.5
    Rotate_cmd.yaw = angle
    rospy.loginfo('Rotating now angle:\n%s',Rotate_cmd.yaw)
    pub_cmd.publish(Rotate_cmd)
    
    if angle == 720:
        angle = 0
        state = 3
        rospy.on_shutdown(Rotate_callback)
        

    if goal.pose.position.x == 0 and goal.pose.position.y == 0 and goal.pose.position.z == 0:
        state = 1

    if goal.pose.position.x == 0 and goal.pose.position.y == 0 and goal.pose.position.z == 0.4:
        state = 2

rospy.init_node('Takeoff_n_Scout')
sub_goal = rospy.Subscriber('/cf1/pose', PoseStamped, Rotate_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=4)
tf_buf   = tf2_ros.Buffer()
tf_lstn  = tf2_ros.TransformListener(tf_buf)

def main():
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if state == 1:
            pub_cmd.publish(takeoff_cmd)
        elif state == 2:
            pub_cmd.publish(Rotate_cmd)
        elif state == 3:
            rospy.on_shutdown('Takeoffnscout')
        # Do something like start exploring
        rate.sleep()

if __name__ == '__main__':
    main()