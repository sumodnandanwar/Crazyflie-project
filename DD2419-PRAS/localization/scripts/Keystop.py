#!/usr/bin/env python
import rospy
import readchar
from std_msgs.msg import String, Empty
from std_msgs.msg import Int8
from crazyflie_driver.msg import Position

k = readchar.readkey()
k = k.lower()

if k == 'w':
    state = 1
    
elif k == 'l':
    state = 2

def main():
    global state
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state == 1:
            #while(1):
            pub_stop.publish(Empty())
            print("Stopped")
        elif state == 2:
            print("Started")                    
            rate.sleep()

pub_stop = rospy.Publisher('/cf1/cmd_stop', Empty, queue_size=1)
rospy.init_node('Emergency_Stop', anonymous=True)

if __name__ == '__main__':
    main()