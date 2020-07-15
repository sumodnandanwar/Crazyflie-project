#!/usr/bin/env python

import sys
import math
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from tf.transformations import euler_from_quaternion

rgoal = None

def goal_callback(msg):
    global rgoal

    # RViz's "2D Nav Goal" publishes z=0, so add some altitude if needed.
    #if msg.pose.position.z == 0.0:
    #    msg.pose.position.z = 0.4

    rospy.loginfo('New goal set:\n%s', msg)
    rgoal = msg

def publish_cmd(rgoal):
    cmd = Position()

    cmd.header.stamp = rospy.Time.now()
    cmd.header.frame_id = rgoal.header.frame_id

    cmd.x = rgoal.pose.position.x
    cmd.x = (10- cmd.x)
    cmd.y = rgoal.pose.position.y
    cmd.y = (10- cmd.y)
    cmd.z = rgoal.pose.position.z

    roll, pitch, yaw = euler_from_quaternion((rgoal.pose.orientation.x,
                                              rgoal.pose.orientation.y,
                                              rgoal.pose.orientation.z,
                                              rgoal.pose.orientation.w))

    cmd.yaw = math.degrees(yaw)

    pub_cmd.publish(cmd)
    #rospy.loginfo('cmd is', cmd)

def main():
    rospy.Rate(20)
    while not rospy.is_shutdown():
        if rgoal:
            publish_cmd(rgoal)
        # else:
        #     print('Final pose (x,y,z,yaw(deg)) = ')
        #     kgoal = input()
        #     pgoal = kgoal.split(',') 
        #     pgoal[0] = float(pgoal[0])
        #     pgoal[1] = float(pgoal[1])
        #     pgoal[2] = float(pgoal[2])
        #     pgoal[3] = float(pgoal[3])
        #     pub_cmd.publish(pgoal)
        #     rospy.loginfo('pgoal is', pgoal)
    rospy.sleep(1)

rospy.init_node('ReadFinalgoal')
sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
pub_cmd = rospy.Publisher('/final_goal', Position, queue_size = 2)

if __name__ == '__main__':
    main()
