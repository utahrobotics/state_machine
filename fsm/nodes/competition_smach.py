#!/usr/bin/env python
from __future__ import division
import math
import rospy
import tf
import actionlib

from std_msgs.msg import Empty

from geometry_msgs.msgs import Twist
from smach import State,StateMachine, Concurrence
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from amee_controllers.msg import CoordinateAction, CoordinateGoal


class Blah(State):
    def __init__(self):
        State.__init__(self, outcomes='success')
    def execute(self):
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        listener = tf.TransformListner()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown()
            try (trans, rot) = listener.lookupTransform('odom', 'map'):
                if trans.x != 0.0
                    a = Twist()
                    a.angular.z = 1.0
                    cmd_pub.publish(a)
                else:
                    break
            rate.sleep()
        return 'success'

# gets called when ANY child state terminates.
# If it returns True, it terminates all concurrent states
def autonomy_child_term_cb(outcome_map):
    """If the teleop toggle is flipped, or the autonomy aborted, terminate all states,
    else wait for all states to finish"""
    if outcome_map['TOGGLE_LISTEN'] == 'invalid':
        return True
    elif outcome_map['AUTONOMY'] == 'aborted':
        return True
    else:
        return False

# gets called when ALL childs terminate
def autonomy_out_cb(outcome_map):
    if outcome_map['TOGGLE_LISTEN'] == 'invalid':
        return 'enter_teleop'
    elif outcome_map['AUTONOMY'] == 'aborted':
        return 'aborted'
    else:
        return 'stay'

def monitor_cb(ud, msg):
    """Callback for the MonitorStates, listening to /click/start_button"""
    # Return False when you want the MonitorState to terminate
    return False

def create_coordinate_goal(name):
    """Get the coordinate goal positions from the configuration file
    and return a CoordinateGoal with these fields"""
    pose = poses[name]

    coordinate_goal = CoordinateGoal()
    coordinate_goal.arm = pose['arm']
    coordinate_goal.bucket = pose['bucket']
    coordinate_goal.sled = pose['sled']
    return coordinate_goal

def create_move_goal(name):
    """Get the move goal params from the configuration file and return
    a MoveBaseGoal with these fields"""
    move = moves[name]
    position = move['position']
    deg = move['deg']

    yaw = deg * math.pi/180
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)

    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.stamp = rospy.Time.now()
    move_goal.target_pose.header.frame_id = 'base_link'
    move_goal.target_pose.pose.position.x = position['x']
    move_goal.target_pose.pose.position.y = position['y']
    move_goal.target_pose.pose.orientation.x = orientation[0]
    move_goal.target_pose.pose.orientation.y = orientation[1]
    move_goal.target_pose.pose.orientation.z = orientation[2]
    move_goal.target_pose.pose.orientation.w = orientation[3]

    return move_goal


def create_action_state(step):
    """Take key value pair of the state of the state and create a SimpleActionState
    based on the parameters in the config file"""
    action_type = step.keys()[0]
    name = step[action_type]
    if action_type == "pose":
        action_state = SimpleActionState('coordinate_action', CoordinateAction, goal=create_coordinate_goal(name))
    elif action_type == "move":
        action_state = SimpleActionState('move_base', MoveBaseAction, goal=create_move_goal(name))
    else:
        raise Exception("action_type pose and move are the only ones implemented")

    rospy.logdebug("State with name")
    return name.upper(), action_state


# where the magic happens
def main():
    smach_states = rospy.get_param("smach_states")
    # The autonomy_sm handles the full autonomy sequence for the competition run
    autonomy_sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
    with autonomy_sm:
        # TODO: create localization state. might just be a MonitorState

        # Sequentially add all the states from the config file to the state machine,
        # where state i transitions to state i+1
        names = []
        for i in range(len(smach_states)):
            state = smach_states[i]
            name, action_state = create_action_state(state)

            # If the state name is already in the state machine, add the new
            # one with increasing numbers
            if name in names:
                name = name+"2"
                while name in names:
                    name = name[:-1]+str(int(name[-1])+1) # increase num
            names.append(name)

            StateMachine.add_auto(name, action_state, connector_outcomes=['succeeded'])

        # StateMachine.add()

    # Create the concurrence contained for the fully autonomy sequence. This
    # runs the state machine for the competition run.  It also concurrently runs
    # a state with a timer counting down from 10 minutes and a state that listens
    # to the /click/start_button topic. If either of these are triggered, it will
    # end autonomy and place us into the teleop state.
    # TODO: add 10 minute competition timer state
    autonomy_concurrence = Concurrence(outcomes=['enter_teleop', 'stay', 'aborted'],
                            default_outcome='enter_teleop',
                            child_termination_cb=autonomy_child_term_cb,
                            outcome_cb=autonomy_out_cb)
    with autonomy_concurrence:
        # state that runs full autonomy state machine
        Concurrence.add('AUTONOMY', autonomy_sm)
        # state that listens for toggle message
        Concurrence.add('TOGGLE_LISTEN', MonitorState('/click/start_button', Empty, monitor_cb))

    # Top level state machine, containing the autonomy and teleop machines.
    top_sm = StateMachine(outcomes=['DONE'])
    with top_sm:
        StateMachine.add('TELEOP_MODE', MonitorState('/click/start_button', Empty, monitor_cb), transitions={'invalid':'AUTONOMY_MODE', 'valid':'TELEOP_MODE', 'preempted':'AUTONOMY_MODE'})
        StateMachine.add('AUTONOMY_MODE', autonomy_concurrence,
          transitions={'enter_teleop':'TELEOP_MODE', 'stay':'AUTONOMY_MODE', 'aborted':'DONE'})

        #StateMachine.add('TELEOP_MODE', MonitorState('/click/start_button', Empty, monitor_cb),
        #  transitions={'invalid':'DONE', 'valid':'TELEOP_MODE', 'preempted':'DONE'})

    sis = IntrospectionServer('smach_introspection_server', top_sm, '/COMPETITION_SMACH')
    sis.start()
    top_sm.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":
    rospy.init_node('competition_smach')#,log_level=rospy.DEBUG)
    # rospy.init_node('competition_smach', log_level=rospy.DEBUG)
    global poses # poses from config file
    global moves # locations to move to in config file
    mining_params = rospy.get_param('/mining_params')
    poses = mining_params["poses"]
    moves = mining_params["moves"]

    main()
