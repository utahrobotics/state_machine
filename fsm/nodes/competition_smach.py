#!/usr/bin/env python
from __future__ import division
import math
import rospy
import tf
import tf2_ros
import actionlib

from std_msgs.msg import Empty, String

from geometry_msgs.msg import Twist, Vector3, Quaternion
import smach
from smach import State, StateMachine, Concurrence, CBState 
from smach_ros import SimpleActionState, MonitorState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from topic_tools.srv import MuxSelect


@smach.cb_interface(outcomes=['succeeded'])
def localization_cb(ud):
    # spin until we get the transform between map and odom 
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    cmd_vel_pub = rospy.Publisher('/hardcode_cmd_vel', Twist, queue_size=10)

    #last_mux = rospy.wait_for_message('/mux_cmd_vel/selected', String).data

    # Move mux to listen to the messages we are about to publish
    mux_select('/hardcode_cmd_vel')
    transform = None

    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        lin = Vector3()
        ang = Vector3(0, 0, 0.2)
        spin_message = Twist(lin, ang)
        cmd_vel_pub.publish(spin_message)

        # if we get the tf, break. else keep looping
        try:
            transform = tfBuffer.lookup_transform('map', 'base_link', rospy.Time.now()-rospy.Duration(0.1))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        rate.sleep()
    mux_select('/move_base_cmd_vel')

    #trans = transform.transform.translation
    #create_move_goal(trans.x, trans.y, 0.0)

    return 'succeeded'

def main():
    # The autonomy_sm handles the full autonomy sequence for the competition run
    autonomy_sm = StateMachine(outcomes=['succeeded','aborted','preempted'])
    with autonomy_sm:
        # Add states for setup (these run once each) 
        for i in range(len(setup_sequence)):
            name, action_type = get_name_and_type(setup_sequence[i])

            func_to_run = None
            if action_type == 'localize':
                func_to_run = localization_cb
            elif action_type == 'move':
                # action instead of function
                func_to_run = 'action'
            else:
                raise NotImplementedError('Only localize is supported rn')

            # if this is the last setup state
            if i == len(loop_sequence)-1:
                # tie the last setup state to the first loop state
                first_loop_name = get_name_and_type(loop_sequence[0])
                if func_to_run == 'action':
                    action_state = create_action_state(name, action_type)
                    StateMachine.add(name, action_state, transitions={'succeeded': first_loop_name})
                else:
                    StateMachine.add(name, CBState(func_to_run), {'succeeded': first_loop_name})
            else:
                # add_auto adds this to the state machine and automatically 
                # ties the next and previous states together
                if func_to_run == 'action':
                    action_state = create_action_state(name, action_type)
                    StateMachine.add_auto(name, action_state, connector_outcomes=['succeeded'])
                else:
                    StateMachine.add_auto(name, CBState(func_to_run), connector_outcomes=['succeeded'])

        names = [] # store names to check and handle dupes
        for i in range(len(loop_sequence)):
            name, action_type = get_name_and_type(loop_sequence[i])
            action_state = create_action_state(name, action_type)

            # "move" states all have the same name and smach requires unique 
            # state names, so check and add a number to the name if needed
            if name in names:
                name = name+"2"
                while name in names:
                    name = name[:-1]+str(int(name[-1])+1) # increase num by 1
            names.append(name)

            if i == len(loop_sequence)-1:
                # tie the last state to the first one, so they just keep looping
                StateMachine.add(name, action_state, transitions={'succeeded': names[0]})
            else:
                # add_auto adds this to the state machine and automatically 
                # ties the next and previous states together
                StateMachine.add_auto(name, action_state, connector_outcomes=['succeeded'])

    # Create the concurrence container for the fully autonomy sequence. This 
    # runs the state machine for the competition run.  It also concurrently runs 
    # a state that listens to the /click_start_button topic. If this is 
    # triggered place us into the teleop state.
    autonomy_concurrence = Concurrence(outcomes=['enter_teleop', 'stay', 'aborted'],
                            default_outcome='enter_teleop',
                            child_termination_cb=autonomy_child_term_cb,
                            outcome_cb=autonomy_out_cb)

    with autonomy_concurrence:
        # state that runs full autonomy state machine
        Concurrence.add('AUTONOMY', autonomy_sm)
        # state that listens for toggle message
        Concurrence.add('TOGGLE_LISTEN', MonitorState('/click_start_button', Empty, start_btn_cb))

    teleop_concurrence = Concurrence(outcomes=['enter_autonomy', 'stay', 'done'],
                            default_outcome='enter_autonomy',
                            child_termination_cb=teleop_child_term_cb,
                            outcome_cb=teleop_out_cb)

    with teleop_concurrence:
        Concurrence.add('TOGGLE_LISTEN', MonitorState('/click_start_button', Empty, start_btn_cb))
        Concurrence.add('EXIT_LISTEN', MonitorState('/click_select_button', Empty, select_btn_cb))


    # Top level state machine, containing the autonomy and teleop machines.
    top_sm = StateMachine(outcomes=['DONE'])
    with top_sm:
        StateMachine.add('TELEOP_MODE', teleop_concurrence,
          transitions={'enter_autonomy':'AUTONOMY_MODE', 'stay':'TELEOP_MODE', 'done':'DONE'})
        StateMachine.add('AUTONOMY_MODE', autonomy_concurrence,
          transitions={'enter_teleop':'TELEOP_MODE', 'stay':'AUTONOMY_MODE', 'aborted':'DONE'})

        #StateMachine.add('TELEOP_MODE', MonitorState('/click_start_button', Empty, monitor_cb),
        #  transitions={'invalid':'DONE', 'valid':'TELEOP_MODE', 'preempted':'DONE'})

    sis = IntrospectionServer('smach_introspection_server', top_sm, '/COMPETITION_SMACH')
    sis.start()
    top_sm.execute()
    rospy.spin()
    sis.stop()


def get_name_and_type(keyval):
    """Take keyval from config file, return capitalized name and action_type"""
    action_type = keyval.keys()[0]
    name = keyval[action_type].upper()
    return name, action_type

# gets called when ANY child state terminates.
# If it returns True, it terminates all concurrent states
def autonomy_child_term_cb(outcome_map):
    """
    If the teleop toggle is flipped, or the autonomy aborted, terminate all 
    states, else wait for all states to finish

    return True means terminate all siblings
    return False means keep going
    """
    if outcome_map['TOGGLE_LISTEN'] == 'invalid':
        return True
    elif outcome_map['AUTONOMY'] == 'aborted':
        return True
    else:
        return False

# gets called when ALL childs terminate
def autonomy_out_cb(outcome_map):
    """Returns transition"""
    if outcome_map['TOGGLE_LISTEN'] == 'invalid':
        return 'enter_teleop'
    elif outcome_map['AUTONOMY'] == 'aborted':
        return 'aborted'
    else:
        return 'stay'

def teleop_child_term_cb(outcome_map):
    """
    If the teleop toggle is flipped, or the autonomy aborted, terminate all 
    states, else wait for all states to finish

    return True means terminate all siblings
    return False means keep going
    """
    if outcome_map['TOGGLE_LISTEN'] == 'invalid':
        return True
    elif outcome_map['EXIT_LISTEN'] == 'invalid':
        return True
    else:
        return False

def teleop_out_cb(outcome_map):
    """Returns transition for end of teleop concurrence"""
    if outcome_map['EXIT_LISTEN'] == 'invalid':
        return 'done'
    elif outcome_map['TOGGLE_LISTEN'] == 'invalid':
        return 'enter_autonomy'
    else:
        return 'stay'

# Flag to keep track of whether teleop or autonomy have control of motors
teleop_mode = True 
def start_btn_cb(ud, msg):
    """Callback for the MonitorStates, triggered on a /click_start_button message"""
    global teleop_mode # (global means make variable read + write from this function)
    teleop_mode = not teleop_mode

    if teleop_mode:
        mux_select('/joystick_cmd_vel')
    else:
        # NOTE: this will always place the state machine into the first state
        mux_select('/move_base_cmd_vel')

    # Return False when you want the MonitorState to terminate, True keeps listening 
    return False 


last_select_btn_time = rospy.Time(0)
def select_btn_cb(ud, msg):
    """Brings down this node, by double tapping the select button (two quick
    publishes of the /click_select_button messages)"""
    global last_select_btn_time # last time the select button was clicked
    now = rospy.Time.now()

    if (now - last_select_btn_time).to_sec() < 0.2:
        # double tap. time to exit 
        return False 
    else:
        last_select_btn_time = now 
        return True

def create_move_goal(x, y, yaw):
    """ """
    orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
    move_goal = MoveBaseGoal()
    move_goal.target_pose.header.stamp = rospy.Time.now()
    move_goal.target_pose.header.frame_id = 'map'
    move_goal.target_pose.pose.position.x = x
    move_goal.target_pose.pose.position.y = y
    move_goal.target_pose.pose.orientation = Quaternion(*orientation)
    return move_goal


def create_action_state(name, action_type):
    """
    Take key value pair of the state of the state and create a 
    SimpleActionState based on the parameters in the config file
    """
    if action_type == "move":
        # Get the move goal params from the configuration file and return
        # a MoveBaseGoal with these fields
        move = moves[name.lower()]
        position = move['position']
        deg = move['deg']
        yaw = deg * math.pi/180
        x, y = position['x'], position['y']
        action_state = SimpleActionState('move_base', MoveBaseAction, goal=create_move_goal(x, y, yaw))
    else:
        raise NotImplentedError("only move actions have been implemented")

    rospy.logdebug("State with name")
    return action_state




if __name__== '__main__':
    rospy.init_node('competition_smach')#,log_level=rospy.DEBUG)
    # select between different topics to route to actual motors
    mux_select = rospy.ServiceProxy('/mux_cmd_vel/select', MuxSelect)
    smach_config = rospy.get_param('/smach_config')
    setup_sequence = smach_config['setup_sequence']
    loop_sequence = smach_config['loop_sequence']
    moves = smach_config['mining_params']['moves'] # locations to move to in config file
    main()
