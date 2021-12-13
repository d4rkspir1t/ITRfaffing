#!/usr/bin/env python2.7
import rospy
import sys
import smach
# from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from smach_ros import *
from move_base_msgs.msg import MoveBaseAction
from vacuum_smach_states import CheckAndMoveSM, CheckAndTurnSM, MoveUntilObstacleState, CheckChargingState
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Bool, Header
from move_base_msgs.msg import MoveBaseGoal


class VacuumCleanerSMNode:
    def __init__(self):
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def create_sm(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        with sm:
            # Version A: using the CheckAndMoveSM
            smach.StateMachine.add('MOVING', CheckAndMoveSM(),
                                   transitions={'obstacle': 'TURNING',
                                                'dirt': 'SPIRALLING',
                                                'low_battery': 'GO_TO_CHARGER'})

            # Option B: Using the MoveUntilObstacleState. Uncomment any, they should be equivalent
            # smach.StateMachine.add('MOVING', MoveUntilObstacleState(),
            #                        transitions={'obstacle': 'TURNING',
            #                                     'dirt': 'SPIRALLING',
            #                                     'low_battery': 'GO_TO_CHARGER'})

            smach.StateMachine.add('TURNING', CheckAndTurnSM(),
                                   transitions={'obstacle': 'TURNING',
                                                'no_obstacle': 'MOVING'})

            # THis could also be a sub state machine hat checks for dirt and spirals
            smach.StateMachine.add('SPIRALLING', SpirallingState(), transitions={'no_dirt': 'MOVING'})

            charger_goal = MoveBaseGoal()  # We always go to the same place, so we use a static goal
            charger_goal.target_pose = PoseStamped(Header(frame_id='map'), Pose(Point(7.809, 4.161, 0),
                                                                                Quaternion(0, 0, 1, 0)))
            smach.StateMachine.add("GO_TO_CHARGER", SimpleActionState('/move_base', MoveBaseAction,
                                                                      goal=charger_goal),
                                   transitions={'succeeded': 'CHARGING', 'aborted': 'MOVING', 'preempted': 'MOVING'})

            smach.StateMachine.add("CHARGING", CheckChargingState(), transitions={'charged': 'MOVING',
                                                                                  'not_charged': 'CHARGING'})
        return sm

    def execute_sm(self):
        sm = self.create_sm()
        sis = IntrospectionServer('tea_Server', sm, 'SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome


# I added this state here to show you can also put it there. It would be nicer in the imported python file.
class SpirallingState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['no_dirt'])
        self.robot_move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.dirt_found = False  # This must be set before the subscriber otherwise it may not work!
        self.dirt_subs = rospy.Subscriber('/dirt', Bool, self.dirt_cb)

    def execute(self, userdata):
        rospy.loginfo('Executing state SPIRALLING')
        d = -1
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.dirt_found:
            d = -d
            for i in range(50):  # Send N messages, arbitrarily
                twist = Twist()
                twist.linear.x = 0.15
                twist.angular.z = d*3
                self.robot_move_pub.publish(twist)
                rate.sleep()
        return 'no_dirt'

    def dirt_cb(self, msg):
        self.dirt_found = msg.data

if __name__ == "__main__":
    rospy.init_node("go_for_tea_sm", sys.argv)
    vacuum = VacuumCleanerSMNode()
    vacuum.execute_sm()
    rospy.spin()
