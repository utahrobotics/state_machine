#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib

from std_msgs.msg import Empty

import smach
from smach_ros import SimpleActionState
from amee_controllers.msg import CoordinateGoal, CoordinateAction


def monitor_cb(ud, msg):
    return False

def main():
    # rospy.init_node('competition_smach')
    rospy.init_node('competition_smach', log_level=rospy.DEBUG)

    # Top level state machine. Seperates the autonomy from the teleop machines.
    sm_top = smach.StateMachine(outcomes=['DONE'])
    with sm_top:
        smach.StateMachine.add('AUTONOMY', smach_ros.MonitorState('/teleop_toggle', Empty, monitor_cb), transitions={'invalid':'TELEOP', 'valid':'AUTONOMY', 'preempted':'AUTONOMY'})
        smach.StateMachine.add('TELEOP', smach_ros.MonitorState('/teleop_toggle', Empty, monitor_cb), transitions={'invalid':'AUTONOMY', 'valid':'TELEOP', 'preempted':'TELEOP'})

    # sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/COMPETITION_SMACH')
    # sis.start()
    sm_top.execute()
    rospy.spin()
    # sis.stop()
    sm_top.request_preempt()

if __name__=="__main__":
    main()
