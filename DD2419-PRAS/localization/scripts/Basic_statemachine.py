#!/usr/bin/env python
import rospy
import smach
import smach_ros 
from smach import CBState
from crazyflie_driver.msg import Position
from geometry_msgs.msg import PoseStamped

angle = 0

@smach.cb_interface(input_keys=['sub_pose'], output_keys=[], outcomes=['finished','failed'])
def takeoff_cb(pose):
    rospy.loginfo('Taking Off')
    takeoff_topic = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    rospy.sleep(1)
    origin = Position()
    origin.header.frame_id = 'cf1/odom'
    origin.x = 0.0
    origin.y = 0.0
    origin.z = 0.4
    origin.yaw = 0
    takeoff_topic.publish(origin)

    if pose.pose.position.x == 0 and pose.pose.position.y == 0 and pose.pose.position.z == 0.4:
        return 'finished'
    else:
        return 'failed'

@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished','failed'])
def rotate_cb( pose):
    rospy.loginfo('Rotate')
    land_topic = rospy.Publisher('/cf1/cmd_position', Position, queue_size=3)
    rospy.sleep(1)
    Rotate_cmd = Position()
    Rotate_cmd.header.frame_id = 'cf1/odom'  

    angle += 2 
    Rotate_cmd.x = 0.0
    Rotate_cmd.y = 0.0
    Rotate_cmd.z = 0.5
    Rotate_cmd.yaw = angle
    land_topic.publish(Rotate_cmd)
    if angle == 360:
        return 'finished'
    else:
        return 'failed'

sub_pose = rospy.Subscriber('/cf1/pose', PoseStamped, takeoff_cb)

if __name__ == '__main__':
    
    rospy.init_node('drone_Takeoff_machine')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['I_have_finished'])
    sm.userdata.lspeed = 0.5
    sm.userdata.rspeed = 1.2
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('drone_takeoff_server', sm, '/SM_DRONE')
    sis.start()

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('TAKEOFF', CBState(takeoff_cb),
                               {'finished': 'ROTATE', 'failed':'I_have_finished'})
        smach.StateMachine.add('ROTATE', CBState(rotate_cb),
                               {'finished': 'I_have_finished', 'failed':'I_have_finished'})
        
    # Execute SMACH plan
    outcome = sm.execute()
    
    sis.stop()