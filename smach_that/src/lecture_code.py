#!/usr/bin/env python
import rospy
import sys
import smach
from std_msgs.msg import String
from smach_ros import IntrospectionServer, ServiceState, SimpleActionState
# from itr_online_tutorials.srv import Str2Coord, Str2CoordRequest
from move_base_msgs.msg import MoveBaseAction


class SM_handler:
    def __init__(self):
        sm = self.create_state_machine()
        sis = IntrospectionServer('go_tea_server', sm, '/SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()

    def create_state_machine(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        with sm:
            with sm:
                smach.StateMachine.add('MOVING',
                                       SimpleActionState('move_base', MoveBaseAction, goal_slots=['target_pose']),
                                       transitions={'succeeded': 'MOVING', 'aborted': 'WAIT_FOR_REQUEST',
                                                    'preempted': 'aborted'},
                                       remapping={'target_pose': 'kitchen_coords'}
                                       transitions={'found_obstacle': 'TURNING',
                                                    'found_dirt': 'DIRT'})

                smach.StateMachine.add('GET_KITCHEN_COORDS',
                                       ServiceState('/str_to_coord', Str2Coord, request=Str2CoordRequest('kitchen'),
                                                    response_slots=['coordinates']),
                                       transitions={'succeeded': 'GO_TO_KITCHEN',
                                                    'preempted': 'aborted'},
                                       remapping={'coordinates': 'kitchen_coords'})

                smach.StateMachine.add('GO_TO_KITCHEN',
                                       SimpleActionState('move_base', MoveBaseAction, goal_slots=['target_pose']),
                                       transitions={'succeeded': 'WAIT_FOR_REQUEST', 'aborted': 'WAIT_FOR_REQUEST',
                                                    'preempted': 'aborted'},
                                       remapping={'target_pose': 'kitchen_coords'})
            return sm


class MovingSt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle', 'dirt', 'low_battery'])
        self.env_said = ''

    def execute(self, userdata):
        rospy.loginfo('Moving around')
        if self.env_said == 'obst':
            return 'obstacle'
        if self.env_said == 'drt':
            return 'dirt'
        if self.env_said == 'lbat':
            return 'low_battery'


class TurningSt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['obstacle', 'n_obstacle'])
        self.env_said = ''

    def execute(self, userdata):
        rospy.loginfo('Moving around')
        if self.env_said == 'obst':
            return 'obstacle'
        if self.env_said == 'n_obst':
            return 'n_obstacle'


class SpiralingSt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['n_dirt'])
        self.env_said = ''

    def execute(self, userdata):
        rospy.loginfo('Vacuuming in a spiral')
        if self.env_said == 'clean':
            return 'n_dirt'


class NavToChSt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_charger'])
        self.env_said = ''

    def execute(self, userdata):
        rospy.loginfo('Going to charger')
        if self.env_said == 'charger':
            return 'at_charger'


class ChargingSt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['charged'])
        self.env_said = ''

    def execute(self, userdata):
        rospy.loginfo('Charging')
        if self.env_said == 'full':
            return 'charged'


if __name__ == '__main__':
    rospy.init_node('vacuum_robot_sm', sys.argv)
    smh = SM_handler()
    rospy.spin()
