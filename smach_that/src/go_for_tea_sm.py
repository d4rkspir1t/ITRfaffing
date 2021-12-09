#!/usr/bin/env python
import rospy
import sys
import smach
from std_msgs.msg import String
from smach_ros import IntrospectionServer, ServiceState, SimpleActionState
from itr_online_tutorials.srv import Str2Coord, Str2CoordRequest
from move_base_msgs.msg import MoveBaseAction


class WaitForRequestState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['received_request', 'req_not_received'], output_keys=['received_request_place_out'])
        self.req_subs = rospy.Subscriber('/get_tea', String, self.request_tea_cb, queue_size=1)
        self.tea_request = None  # FIXME change this to a queue!

    def request_tea_cb(self, msg):
        self.tea_request = msg.data
        rospy.loginfo("I received a request to go to " + self.tea_request)

    def execute(self, userdata):
        rospy.loginfo('Executing State Wait For Request')
        if self.tea_request:
            # WE HAVE TO GO SOMEWHERE
            userdata.received_request_place_out = self.tea_request
            self.tea_request = None
            return 'received_request'
        else:
            rospy.sleep(0.5)
            return 'req_not_received'

class GoForTeaSMNode:
    def __init__(self):
        pass

    def create_state_machine(self):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        with sm:
            smach.StateMachine.add('WAIT_FOR_REQUEST', WaitForRequestState(), transitions={'received_request': 'GET_KITCHEN_COORDS',
                                                                                           'req_not_received': 'WAIT_FOR_REQUEST'})

            smach.StateMachine.add('GET_KITCHEN_COORDS', ServiceState('/str_to_coord', Str2Coord, request=Str2CoordRequest('kitchen'),
                                                                      response_slots=['coordinates']),
                                   transitions={'succeeded': 'GO_TO_KITCHEN',
                                                'preempted': 'aborted'},
                                   remapping={'coordinates': 'kitchen_coords'})

            smach.StateMachine.add('GO_TO_KITCHEN', SimpleActionState('move_base', MoveBaseAction, goal_slots=['target_pose']),
                                   transitions={'succeeded': 'WAIT_FOR_REQUEST', 'aborted': 'WAIT_FOR_REQUEST',
                                                'preempted': 'aborted'},
                                   remapping={'target_pose': 'kitchen_coords'})
            #smach.StateMachine.add('BRING_TEA', XXX)
            #smach.StateMachine.add('GO_TO_REST', XXX)

        return sm

    def run_sm(self):
        sm = self.create_state_machine()
        sis = IntrospectionServer('go_tea_server', sm, '/SM_ROOT')
        sis.start()
        outcome = sm.execute()
        sis.stop()
        return outcome


if __name__ == "__main__":
    rospy.init_node('go_for_tea_sm', sys.argv)
    gftn = GoForTeaSMNode()
    gftn.run_sm()
    rospy.spin()
