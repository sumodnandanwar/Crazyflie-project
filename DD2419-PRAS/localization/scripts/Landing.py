#!/usr/bin/env python
import math
import rospy
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from tf.transformations import euler_from_quaternion

state = 0
goal = None 

def landing_cmd(goal):
    global state, alt, landing_cmd
    landing_cmd = Position()
    landing_cmd.header.frame_id = 'cf1/odom'
    landing_cmd.x = goal.pose.position.x
    landing_cmd.y = goal.pose.position.y
    alt = goal.pose.position.z
    alt -= 0.1
    landing_cmd.z = alt
    landing_cmd.yaw = goal.pose.orientation.z

    if goal.pose.position.z != 0.000000:
        #Landing executing
        state = 1

    if alt == 0 and goal.pose.position.z == 0.0000:
        #Landing done
        state = 2
        #rospy.on_shutdown(landing_cmd)

    pub_cmd.publish(landing_cmd)



rospy.init_node('Landing')
Path_goal = rospy.Subscriber('/cf1/pose', PoseStamped, landing_cmd)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=5)
rospy.loginfo('Landing alt:\n%s',landing_cmd)

def main():
    rate = rospy.Rate(10)  # Hz
    while not rospy.is_shutdown():
        if state == 1:
            pub_cmd.publish(landing_cmd)
        elif state == 2:
            rospy.on_shutdown()
        rate.sleep()
        
        #    rospy.on_shutdown('Landing')
if __name__ == '__main__':
    main()