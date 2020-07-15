#!/usr/bin/env python

import math
import numpy as np
import rospy
from os.path import expanduser
import csv
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped , PoseWithCovarianceStamped
from crazyflie_driver.msg import Position
from std_msgs.msg import Empty, Int16

# Global variables
state = 0
angle = 0
thr = 0.15
lap = 0
initialize = 0
x =  0
cu_pose = PoseWithCovarianceStamped()
xmean = 0
ymean = 0
cmd = Position()
trig_msg = 0
odom_cmd = Position()

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
        # next(csv_reader)
        for line in csv_reader:
            line[0] = float(line[0])
            line[1] = float(line[1])
            dictgoal.append(line)
    # print dictgoal[0][1]
    return dictgoal

def Trigdata(trig):
    global trig_msg
    trig_msg = trig.data
    print trig_msg

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
    # print xmean, ymean

def originn(xmn,ymn):
    global origin

    origin = Position()
    origin.header.frame_id = 'cf1/odom'
    origin.x = xmn
    origin.y = ymn
    origin.z = 0.4
    origin.yaw = 0
    return origin

def Rotate_callback(rposex,rposey):
    global state, angle, Rotate_cmd, lap

    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 5 
    Rotate_cmd.x = rposex
    Rotate_cmd.y = rposey
    if (cu_pose.pose.pose.position.z>= 0.375 and cu_pose.pose.pose.position.z <= 0.425):
        Rotate_cmd.z = cu_pose.pose.pose.position.z
    else:
        Rotate_cmd.z = 0.4
    # Rotate_cmd.z = 0.4 
    Rotate_cmd.yaw = angle
    if angle == 350:
        angle = 0
        lap += 1
    return Rotate_cmd
    
def Landing_callback(lpose):
    global alt, landing_cmd ,state
    print ('in landing')
    landing_cmd = Position()
    landing_cmd.header.frame_id = 'cf1/odom'
    landing_cmd.x = lpose.pose.pose.position.x
    landing_cmd.y = lpose.pose.pose.position.y
    alt = lpose.pose.pose.position.z
    alt -= 0.1
    landing_cmd.z = alt
    landing_cmd.yaw = 0   
    if alt < 0.15 and lpose.pose.pose.position.z < 0.15:
        state = 7
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

    # tf_buf.lookup_transform()
    roll, pitch, yaww = euler_from_quaternion((goal.pose.orientation.x,
                                              goal.pose.orientation.y,
                                              goal.pose.orientation.z,
                                              goal.pose.orientation.w), axes = "rzyx")
    yaww = round(yaww,3)
                                        
    if goal_odom:
        cmd = Position()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "cf1/odom"
        cmd.x = goal_odom.pose.position.x #+ 0.5
        cmd.y = goal_odom.pose.position.y #+ 0.5
        # cmd.z = 0.4
        if (cu_pose.pose.pose.position.z>= 0.38 and cu_pose.pose.pose.position.z <= 0.42):
            cmd.z = cu_pose.pose.pose.position.z
        else:
            cmd.z = 0.4 

        if state == 5:
            cmd.yaw = -180-(math.degrees(yaww))
        else:
            cmd.yaw = 180
            
        # print cmd.yaw
        odom_cmd = cmd
        return odom_cmd

def sub_callback(cf1_pose):
    global cu_pose
    cu_pose = cf1_pose
    
def state_machine():
    global state, initialize, x, lap, trig_msg
    
    xval = round(cu_pose.pose.pose.position.x,3)    
    yval = round(cu_pose.pose.pose.position.y,3)
    zval = round(cu_pose.pose.pose.position.z,3)
    # print xval, yval, zval

    if initialize < 1 and zval < 0.15:
        state = 1
        initialize += 1

    elif(zval >= 0.375 and zval <= 0.425) and dist(xval,yval,xmean,ymean) < thr and lap == 0:
        state = 2    

    elif state >= 2 and (zval >= 0.375 and zval <= 0.425) and lap >= 1: #Head to waypoint
        Dict_goal()
        if dictgoal and tf_buf.can_transform('map', 'cf1/odom', rospy.Time.now(), timeout=rospy.Duration(0.2)):         
            xcmd = dictgoal[x][0]
            ycmd = dictgoal[x][1]

            if (xcmd !=100.0 and ycmd !=100.0):
                goalpub(xcmd,ycmd)
                print xval,yval,zval,cmd.x,cmd.y,xcmd,ycmd
                if (dist(xval,yval,cmd.x,cmd.y) > thr): #pathpoint not reached
                    state = 3
                    print (dist(xval,yval,cmd.x,cmd.y))
                    print ('Getting to pathpoint', x)
                elif dist(xval,yval,cmd.x,cmd.y) < thr:
                    print ('Reached pathpoint', x)
                    if x != (len(dictgoal)-1):
                        x += 1
                    elif x == (len(dictgoal)-1): #Last pathpoint
                        state = 6
            
            elif (xcmd==100.0 and ycmd==100.0):

                if x != (len(dictgoal)-1):
                    xcmd = dictgoal[x-1][0]
                    ycmd = dictgoal[x-1][1]
                    goalpub(xcmd,ycmd)
                    state = 5   #Publish update odom cmd yaw with previous saved odomx and odom
                    if trig_msg == 1: #Start with the checkpoint
                        lap = 0
                        state = 4
                        x += 1
                        trig_msg = 0
                elif x == (len(dictgoal)-1):  #Last pathpoint
                    state = 6
        else:
            state = 2


rospy.init_node('Takeoff_n_Scout')
sub_pose = rospy.Subscriber('/pose_filtered', PoseWithCovarianceStamped, sub_callback)
pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
initial_pose = rospy.Subscriber('/cf1/pose', PoseStamped, init_pose)
Trig_data = rospy.Subscriber('Object_Trig', Int16, Trigdata)
pub_stop = rospy.Publisher('/cf1/cmd_stop', Empty, queue_size=1)
tf_buf = tf2_ros.Buffer()
tf_lstn = tf2_ros.TransformListener(tf_buf)

def main():
    rate = rospy.Rate(10)  # Hz
    # print ('main')
    while not rospy.is_shutdown():
        state_machine()
        if state == 1:
            originn(xmean,ymean)
            pub_cmd.publish(origin)
        elif state == 2:
            # print state
            Rotate_callback(xmean,ymean)
            pub_cmd.publish(Rotate_cmd)
        elif state == 3:
            # print state            
            pub_cmd.publish(odom_cmd)
        elif state == 4:
            print state            
            Rotate_callback(odom_cmd.x,odom_cmd.y)
            pub_cmd.publish(Rotate_cmd)
        elif state == 5:
            pub_cmd.publish(odom_cmd)
        elif state == 6:
            # print state
            Landing_callback(cu_pose)
            pub_cmd.publish(landing_cmd)
        elif state == 7:
            pub_stop.publish(Empty())
            print('Mission Complete')
        rate.sleep()

if __name__ == '__main__':
    main()