#!/usr/bin/env python

import math
import numpy as np
import rospy
from os.path import expanduser
import csv
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty

state = 0
angle = 0
thr = 0.1
lap = 0
initialize = 0

def Dict_goal():
    global dictgoal
    dictgoal = []
    pathcsv = expanduser('~')
    pathcsv += '/dd2419_ws/src/DD2419-PRAS/localization/scripts/Pathcsv'
    with open(pathcsv, 'rb') as csv_file:
        csv_reader = csv.reader(csv_file)
        next(csv_reader)
        for line in csv_reader:
            line[0] = float(line[0])
            line[1] = float(line[1])
            dictgoal.append(line)
    print dictgoal[0][1]
    return dictgoal

def init_pose(data):
    global xmean, ymean
    patharray = []
    pathxarray = []
    pathyarray = []
    x = 0
    y = 0
    pose = [x,y]  
    for i in range(0,10):
        pose[0] = round(data.pose.position.x,3)
        pose[1] = round(data.pose.position.y,4)
        patharray.append(pose)
        pathxarray.append(pose[0])
        pathyarray.append(pose[1])
        if i == 9:
            initial_pose.unregister()
    xmean = round((np.mean(pathxarray)),4)
    ymean = round((np.mean(pathyarray)),4)
    print xmean, ymean

def originn(xmn,ymn):
    global origin

    origin = Position()
    origin.header.frame_id = 'cf1/odom'
    origin.x = xmn
    origin.y = ymn
    origin.z = 0.4
    origin.yaw = 000
    # origin = pose
    return origin

def setpoint1():
    global spoint1

    spoint1 = Position()
    spoint1.header.frame_id = 'cf1/odom'
    spoint1.x = -0.00 + 0.5
    spoint1.y = -0.4 + 0.5
    spoint1.z = 0.4
    spoint1.yaw = 000
    # spoint = pose
    return spoint1

def setpoint2():
    global spoint2

    spoint2 = Position()
    spoint2.header.frame_id = 'cf1/odom'
    spoint2.x = -0.4 + 0.5
    spoint2.y = -0.4 + 0.5
    spoint2.z = 0.4
    spoint2.yaw = 000
    # spoint = pose
    return spoint2

def setpoint3():
    global spoint3

    spoint3 = Position()
    spoint3.header.frame_id = 'cf1/odom'
    spoint3.x = +0.4 + 0.5
    spoint3.y = -0.4 + 0.5
    spoint3.z = 0.4
    spoint3.yaw = 000
    # spoint = pose
    return spoint3

def setpoint4():
    global spoint4

    spoint4 = Position()
    spoint4.header.frame_id = 'cf1/odom'
    spoint4.x = +0.4 + 0.5
    spoint4.y = +0.4 + 0.5
    spoint4.z = 0.4
    spoint4.yaw = 000
    # spoint = pose
    return spoint4

def setpoint5():
    global spoint5

    spoint5 = Position()
    spoint5.header.frame_id = 'cf1/odom'
    spoint5.x = -0.4 + 0.5
    spoint5.y = 0.4 + 0.5
    spoint5.z = 0.4
    spoint5.yaw = 000
    # spoint = pose
    return spoint5


def Rotate_callback(origin):
    global state, angle, Rotate_cmd, lap

    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 5 
    Rotate_cmd.x = origin.x
    Rotate_cmd.y = origin.y
    Rotate_cmd.z = origin.z
    Rotate_cmd.yaw = angle

    if angle == 360:
        angle = 0
        lap += 1
        # print state
    return Rotate_cmd
    
def Landing_callback(lpose):
    global alt, landing_cmd, state
    print ('in landing')
    landing_cmd = Position()
    landing_cmd.header.frame_id = 'cf1/odom'
    landing_cmd.x = lpose.pose.position.x
    landing_cmd.y = lpose.pose.position.y
    alt = lpose.pose.position.z
    alt -= 0.1
    landing_cmd.z = alt
    landing_cmd.yaw = 0   
    if alt < 0.15 and lpose.pose.position.z < 0.15:
        state = 11
        # landing_cmd = None
    return landing_cmd


def dist(myposex, myposey, goalx, goaly):
    dis = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2)
    return dis

def goalpub(cmdx,cmdy):
    global odom_cmd, cmd

    # rate = rospy.Rate(10)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = "map"
    goal.pose.position.x = cmdx
    goal.pose.position.y = cmdy
    goal.pose.position.z = 0.4
    if not tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):
        rospy.logwarn_throttle(10.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
        return

    goal_odom = tf_buf.transform(goal, 'cf1/odom')

    if goal_odom:
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "cf1/odom"
        cmd.x = goal_odom.pose.position.x - 0.5
        cmd.y = goal_odom.pose.position.y - 0.5
        cmd.z = goal_odom.pose.position.z
        cmd.yaw = 180
        odom_cmd = cmd
        # print odom_cmd
        return odom_cmd


def state_machine(cf1_pose):
    global cu_pose, state, initialize, xval, yval, zval
    cu_pose = cf1_pose
    xval = round(cu_pose.pose.position.x,2)    
    yval = round(cu_pose.pose.position.y,2)
    zval = round(cu_pose.pose.position.z,2)
    # print xval, yval, zval
     
    if initialize < 1 and zval < 0.15:
        state = 1
        print zval
        initialize += 1
    if  zval == 0.4 and dist(xval,yval,xmean,ymean) < thr and lap == 0:
        state = 2
    if dist(xval,yval,xmean,ymean) < thr and lap == 1 :
        state = 3
    if  state >= 3 and zval == 0.4 and dist(xval,yval,spoint1.x,spoint1.y) < thr:
        state = 4
    if  state >= 4 and zval == 0.4 and dist(xval,yval,spoint2.x,spoint2.y) < thr:
        state = 5
    if dist(xval,yval,spoint2.x,spoint2.y) < thr and lap == 2:
        state = 6
    if  state >= 6 and zval == 0.4 and dist(xval,yval,spoint3.x,spoint3.y) < thr:
        state = 7
    if  state >= 7 and zval == 0.4 and dist(xval,yval,spoint4.x,spoint4.y) < thr:
        state = 8
    if  state >= 8 and zval == 0.4 and dist(xval,yval,spoint5.x,spoint5.y) < thr:
        state = 9
    if dist(xval,yval,spoint5.x,spoint5.y) < thr and lap == 3:
        state = 10


rospy.init_node('Takeoff_n_Scout')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, state_machine)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
pub_stop = rospy.Publisher('/cf1/cmd_stop', Empty, queue_size=1)
initial_pose = rospy.Subscriber('/cf1/pose', PoseStamped, init_pose)
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)

def main():
    Dict_goal()
    setpoint1()
    setpoint2()
    setpoint3()
    setpoint4()
    setpoint5()
    rate = rospy.Rate(10)  # Hz
    print ('main')
    while not rospy.is_shutdown():
        if state == 1:
            originn(xmean,ymean)
            pub_cmd.publish(origin)
        elif state == 2:
            Rotate_callback(origin)
            pub_cmd.publish(Rotate_cmd)
            print('Spinning')
        elif state == 3:
            pub_cmd.publish(spoint1)
            print('Setpoint1')
        elif state == 4:
            pub_cmd.publish(spoint2)
            print('Setpoint2')
        elif state == 5:
            Rotate_callback(spoint2)
            pub_cmd.publish(Rotate_cmd)
            print('Spinning')
        elif state == 6:
            pub_cmd.publish(spoint3)
            print('Setpoint3')
        elif state == 7:
            pub_cmd.publish(spoint4)
            print('Setpoint4')
        elif state == 8:
            pub_cmd.publish(spoint5)
            print('Setpoint5')
        elif state == 9:
            Rotate_callback(spoint5)
            pub_cmd.publish(Rotate_cmd)
            print('Spinning')
        elif state == 10:
            print state
            Landing_callback(cu_pose)
            pub_cmd.publish(landing_cmd)
        elif state == 11:
            pub_stop.publish(Empty())
            print('Mission Complete')
        rate.sleep()

if __name__ == '__main__':
    main()