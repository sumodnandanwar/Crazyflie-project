#!/usr/bin/env python

import roslib; roslib.load_manifest('coffee_state')
import rospy
import smach
import smach_ros
import geometry_msgs
import move_base
import move_base_msgs
import actionlib

from actionlib import SimpleActionClient
from geometry_msgs.msg import *
from move_base_msgs.msg import *

#Goal locations
elevator3_front = PoseStamped()
elevator3_back = PoseStamped()
elevator3_front_end = PoseStamped()
elevator2_front = PoseStamped()
elevator2_back = PoseStamped()
elevator2_front_end = PoseStamped()
cafe = PoseStamped()
labDoor = PoseStamped()
hallway_door = PoseStamped()

client = SimpleActionClient("move_base", MoveBaseAction)

#Initializes the various goal locations from measured data
def init_locations():
    seq = 0

    elevator3_front.header.frame_id = "map"
    elevator3_front.header.seq = seq
    seq += 1
    elevator3_front.pose.position.x = -7.59
    elevator3_front.pose.position.y = 7.58
    elevator3_front.pose.orientation.z = -0.75995236248
    elevator3_front.pose.orientation.w = 0.649978774085

    elevator3_back.header.frame_id = "map"
    elevator3_back.header.seq = seq
    seq += 1
    elevator3_back.pose.position.x = -7.08418655396
    elevator3_back.pose.position.y = 3.93412828445
    elevator3_back.pose.orientation.z = 0.726080312711
    elevator3_back.pose.orientation.w = 0.687609903574
    #elevator3_front.header.seq = 2

    elevator3_front_end.header.frame_id = "map"
    elevator3_front_end.header.seq = seq
    seq += 1
    elevator3_front_end.pose.position.x = -7.51548194885
    elevator3_front_end.pose.position.y = 7.32916355133
    elevator3_front_end.pose.orientation.z = 0.705664372808
    elevator3_front_end.pose.orientation.w = 0.708546253219

    elevator2_front.header.frame_id = "map"
    elevator2_front.header.seq = seq
    seq += 1
    elevator2_front.pose.position.x = 9.57122993469
    elevator2_front.pose.position.y = 35.1978797913
    elevator2_front.pose.orientation.z = -0.338406673277
    elevator2_front.pose.orientation.w = 0.940999959342

    elevator2_back.header.frame_id = "map"
    elevator2_back.header.seq = seq
    seq += 1
    elevator2_back.pose.position.x = 7.59552383423
    elevator2_back.pose.position.y = 36.9895133972
    elevator2_back.pose.orientation.z = -0.405414000525
    elevator2_back.pose.orientation.w = 0.914133189518

    elevator2_front_end.header.frame_id = "map"
    elevator2_front_end.header.seq = seq
    seq += 1
    elevator2_front_end.pose.position.x = 9.58093166351
    elevator2_front_end.pose.position.y = 35.1085357666
    elevator2_front_end.pose.orientation.z = 0.925599866249
    elevator2_front_end.pose.orientation.w = 0.378503484264

    cafe.header.frame_id = "map"
    cafe.header.seq = seq
    seq += 1
    cafe.pose.position.x = -10.9466533661
    cafe.pose.position.y = 43.2356872559
    cafe.pose.orientation.z = -0.397528812737
    cafe.pose.orientation.w = 0.917589692098

    labDoor.header.frame_id = "map"
    labDoor.header.seq = seq
    seq += 1
    labDoor.pose.position.x = 24.8734970093
    labDoor.pose.position.y = 7.63800287247
    labDoor.pose.orientation.w = 1.0

    hallway_door.header.frame_id = "map"
    hallway_door.header.seq = seq
    seq += 1
    hallway_door.pose.position.x = 4.84898424149
    hallway_door.pose.position.y = 8.04237365723
    hallway_door.pose.orientation.z = 1.0



def move(loc,name):
    goal = MoveBaseGoal()
    goal.target_pose = loc
    goal.target_pose.header.stamp = rospy.Time.now()
    #print(goal)
    client.send_goal(goal)
    #print(client.get_result())
    client.wait_for_result(rospy.Duration.from_sec(1000.0))

#Move state to a specified location
class Move(smach.State):
    def __init__(self, loc, name="default"):
        smach.State.__init__(self, outcomes=['done'])
        self.loc = loc
        self.name = name

    def execute(self, userdata):
        move(self.loc,self.name)
        return 'done'

class Wait(smach.State):
    def __init__(self, msg):
        smach.State.__init__(self, outcomes=['done','skip'])
        self.msg = msg

    def execute(self, userdata):
        print self.msg + ":"
        result = raw_input()
        if result.lower() == 's' or result.lower() == 'skip':
            return 'skip'
        else:
            return 'done'

class NavigateElevator(smach.State):
    def __init__(self, back, front):
        smach.State.__init__(self, outcomes=['done'])
        self.front = front
        self.back = back

    def execute(self, userdata):
        print("Please press a key when elevator door is complete open...")
        raw_input()
        move(back, "elevator_back")
        print("Please press a key when elevator is at floor " ++
              "and door is completely open...")
        raw_input()
        move(front, "elevator_front")
        return 'done'

def main():
    rospy.init_node('smach_example_state_machine')

    print ("waiting")
    client.wait_for_server()

    print ("init")
    init_locations()

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['finished'])

    # Open the container
    with sm:
        # Add states to the container
        #smach.StateMachine.add('MOVETOLABDOOR',
        #                       Move(labDoor, "Moving to lab door..."),
        #                       transitions={'done' : 'WAITFORLABDOOR'})
        #smach.StateMachine.add('WAITFORLABDOOR',
        #                       Wait('Please open the lab door and press enter...'),
        #                       transitions={'done' : 'MOVETOHALLWAYDOOR'})
        smach.StateMachine.add('START',
                               Wait('Please press enter to start..'),
                               transitions={'done':'MOVETOHALLWAYDOOR','skip':'WAITHALLWAYDOOR'})
        smach.StateMachine.add('MOVETOHALLWAYDOOR',
                               Move(hallway_door, "hallway_door"),
                               transitions={'done':'WAITHALLWAYDOOR'})
        smach.StateMachine.add('WAITHALLWAYDOOR',
                               Wait('Please open the hallway door and press enter..'),
                               transitions={'done':'MOVETOELEVATOR3','skip':'WAITFORELEVATOR3'})
        smach.StateMachine.add('MOVETOELEVATOR3',
                               Move(elevator3_front, "elevator3_front"),
                               transitions={'done':'WAITFORELEVATOR3'}) # MODIFIED LINE FOR DEMO
        smach.StateMachine.add('WAITFORELEVATOR3',
                               Wait("Please call elevator and press enter..."),
                               transitions={'done':'ENTERELEVATOR3','skip':'RIDEELEVATOR3'})
        smach.StateMachine.add('ENTERELEVATOR3',
                               Move(elevator3_back),
                               transitions={'done':'RIDEELEVATOR3'})
        smach.StateMachine.add('RIDEELEVATOR3',
                               Wait("Please press a key when elevator is at floor 2 and door is completely open..."),
                               transitions={'done':'EXITELEVATOR2', 'skip':'SWITCHTO2'})
        smach.StateMachine.add('EXITELEVATOR2',
                               Move(elevator3_front),
                               transitions={'done':'SWITCHTO2'})
        smach.StateMachine.add('SWITCHTO2',
                               Wait('Please switch maps to the second floor and press a key...'),
                               transitions={'done':'MOVETOCAFE','skip':'TAKECOFFEE'})
        smach.StateMachine.add('MOVETOCAFE', Move(cafe, 'cafe'),
                               transitions={'done':'TAKECOFFEE'})
        smach.StateMachine.add('TAKECOFFEE', Wait('Please place coffee on robot...'),
                               transitions={'done':'MOVETOELEVATOR2', 'skip':'WAITFORELEVATOR2'})
        smach.StateMachine.add('MOVETOELEVATOR2',
                               Move(elevator2_front_end, 'elevator2_front_retrieved'),
                               transitions={'done':'WAITFORELEVATOR2'})
        smach.StateMachine.add('WAITFORELEVATOR2',
                               Wait('Please call the elevator and press a key...'),
                               transitions={'done':'ENTERELEVATOR2', 'skip':'RIDEELEVATOR2'})
        smach.StateMachine.add('ENTERELEVATOR2',
                               Move(elevator2_back),
                               transitions={'done':'RIDEELEVATOR2'})
        smach.StateMachine.add('RIDEELEVATOR2',
                               Wait('Please press a key when elevator is at floor 3 and door is completely open...'),
                               transitions={'done':'EXITELEVATOR3', 'skip':'SWITCHTO3'})
        smach.StateMachine.add('EXITELEVATOR3',
                               Move(elevator3_front_end),
                               transitions={'done':'SWITCHTO3'})
        smach.StateMachine.add('SWITCHTO3',
                               Wait('Please switch maps to the third floor and press a key...'),
                               transitions={'done':'MOVETOLAB','skip':'finished'})
        smach.StateMachine.add('MOVETOLAB', Move(labDoor, "lab"),
                               transitions={'done':'finished'})


    # Create and start the introspection server
    #sis = smach_ros.IntrospectionServer('coffee_state_server', sm, '/SM_ROOT')
    #sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    #sis.stop()

if __name__ == '__main__':
    main()