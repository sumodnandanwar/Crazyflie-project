#!/usr/bin/env python
import rospy
import math
import readchar
from std_msgs.msg import String, Empty, Int16

def main():
    global msg
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        k = readchar.readkey()
        k = k.lower()

        if k == 'd':
            msg = 1
            print('Object Detected')
        else:
            msg = 0
            print('Object Not Detected')

        triggerit = msg
        #print(triggerit)
        pub_trigger.publish(triggerit)
        rate.sleep()

rospy.init_node('Key_Object_Trigger', anonymous=True)
pub_trigger = rospy.Publisher('Object_Trig', Int16 ,queue_size=10)


if __name__ == '__main__':
    main()