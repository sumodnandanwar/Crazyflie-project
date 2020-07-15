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
x =  0

def dist(myposex, myposey, goalx, goaly):
    dis = math.sqrt((myposex-goalx)**2+(myposey-goaly)**2)
    return dis

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
    # print dictgoal[0][1]
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
    origin.yaw = 0
    return origin

# def setpoint1():
#     global spoint1

#     spoint1 = Position()
#     spoint1.header.frame_id = 'cf1/odom'
#     spoint1.x = -0.25
#     spoint1.y = -0.25
#     spoint1.z = 0.4
#     spoint1.yaw = 000
#     # spoint = pose
#     return spoint1


def Rotate_callback(origin):
    global state, angle, Rotate_cmd, lap

    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 5 
    Rotate_cmd.x = origin.x
    Rotate_cmd.y = origin.y
    Rotate_cmd.z = origin.z
    Rotate_cmd.yaw = angle
    # rospy.loginfo('Rotating now angle:\n%s',Rotate_cmd.yaw)
    # pub_cmd.publish(Rotate_cmd)
    if angle == 360:
        angle = 0
        lap += 1
    return Rotate_cmd
    
def Landing_callback(lpose):
    global alt, landing_cmd
    print ('in landing')
    landing_cmd = Position()
    landing_cmd.header.frame_id = 'cf1/odom'
    landing_cmd.x = lpose.pose.position.x
    landing_cmd.y = lpose.pose.position.y
    alt = lpose.pose.position.z
    alt -= 0.1
    landing_cmd.z = alt
    landing_cmd.yaw = 0   
    if alt == 0 and lpose.pose.position.z == 0.0:
        #Landing done
        landing_cmd = None
    return landing_cmd


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
        #print(goal_odom.header.stamp)
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "cf1/odom"
        cmd.x = goal_odom.pose.position.x + 0.5
        cmd.y = goal_odom.pose.position.y + 0.5
        cmd.z = goal_odom.pose.position.z
        cmd.yaw = 180
        odom_cmd = cmd
        print ('odom_cmd = %s,%s',odom_cmd.x,odom_cmd.y)
        return odom_cmd

# def waypoint_nav():
#     global x, state
#     # for e in range(len(dictgoal)):
#     while True:
#         xcmd = dictgoal[x][0]
#         ycmd = dictgoal[x][1]
#         goalpub(xcmd,ycmd) #Returns cmd.x and cmd.y
#         print xval,yval,cmd.x,cmd.y,xcmd,ycmd
#         if dist(xval,yval,cmd.x,cmd.y) > thr and zval == 0.4: #pathpoint not reached
#             state = 3
#             print ('Getting to pathpoint')
#         elif dist(xval,yval,cmd.x,cmd.y) < thr and zval == 0.4:
#             print ('Reached pathpoint')
#             if x != (len(dictgoal)-1):
#                 x += 1
#             elif x == (len(dictgoal)-1):
#                 state = 4
#                 break
        
            

# def publish_cmd(cmd):
#     if cmd != None:
#         cmd.header.stamp = rospy.Time.now()
#         pub_cmd.publish(cmd)
#     elif cmd == None:
#         # cmd.header.stamp = rospy.Time.now()
#         cmd.z = 0.4
#         pub_cmd.publish(cmd)

def state_machine(cf1_pose):
    global cu_pose, state, initialize, xval, yval, zval , x
    cu_pose = cf1_pose



    xval = round(cu_pose.pose.position.x,3)    
    yval = round(cu_pose.pose.position.y,3)
    zval = round(cu_pose.pose.position.z,3)
    # print xval, yval, zval
     
    if initialize < 1 and zval < 0.15:
        state = 1
        print zval
        initialize += 1

    if  zval == 0.4 and dist(xval,yval,xmean,ymean) < thr and lap == 0:
        state = 2    

    if state >= 2 and zval == 0.4 and dist(xval,yval,xmean,ymean) < thr and lap == 1: #Head to waypoint
        print xval,yval
        while True:
            print ('in while')
            xcmd = dictgoal[x][0]
            ycmd = dictgoal[x][1]
            goalpub(xcmd,ycmd) #Returns cmd.x and cmd.y
            print xval,yval,cmd.x,cmd.y,xcmd,ycmd
            if dist(xval,yval,cmd.x,cmd.y) > thr and zval == 0.4: #pathpoint not reached
                state = 3
                print ('Getting to pathpoint')
            if dist(xval,yval,cmd.x,cmd.y) < thr and zval == 0.4:
                print ('Reached pathpoint')
                if x != (len(dictgoal)-1):
                    x += 1
                elif x == (len(dictgoal)-1):
                    state = 4
                    break

    # for e in range(len(dictgoal)):
    #     xcmd = dictgoal[e][0]
    #     ycmd = dictgoal[e][1]
    #     print xcmd,ycmd
    #     goalpub(xcmd,ycmd)
    #     pub_cmd.publish(odom_cmd)

rospy.init_node('Takeoff_n_Scout')
sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, state_machine)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
initial_pose = rospy.Subscriber('/cf1/pose', PoseStamped, init_pose)
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)

def main():
    Dict_goal()
    rate = rospy.Rate(10)  # Hz
    # print ('main')
    while not rospy.is_shutdown():
        if state == 1:
            originn(xmean,ymean)
            pub_cmd.publish(origin)
            # print state
        elif state == 2:
            Rotate_callback(origin)
            pub_cmd.publish(Rotate_cmd)
        elif state == 3:
            print odom_cmd
            pub_cmd.publish(odom_cmd)
            # setpoint()
            # pub_cmd.publish(spoint)
            # state = 4
            # for e in range(len(dictgoal)):
            #     xcmd = dictgoal[e][0]
            #     ycmd = dictgoal[e][1]
            #     print xcmd,ycmd
            #     goalpub(xcmd,ycmd)
            #     pub_cmd.publish(odom_cmd)
            
        elif state == 4:
            print state
            Landing_callback(cu_pose)
            pub_cmd.publish(landing_cmd)
        rate.sleep()

if __name__ == '__main__':
    main()