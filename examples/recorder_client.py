#! /usr/bin/env python

from __future__ import print_function
import sys
import random

import rospy
import actionlib

from recorder_msgs.msg import DurationRecorderAction, DurationRecorderGoal
from recorder_msgs.msg import RollingRecorderAction, RollingRecorderGoal


NODE_NAME = 'recorder_client'
# Choose 'rolling_recorder' or 'duration_recorder'
recorder_type = sys.argv[1]
record_time = 10    # seconds

if recorder_type == 'rolling_recorder':
    action = '/rolling_recorder/RosbagRollingRecord'
    action_type = RollingRecorderAction
    goal = RollingRecorderGoal(destination='rolling_recorder_test/')
    print('RollingRecorderGoal:')
elif recorder_type == 'duration_recorder':
    action = '/duration_recorder/RosbagDurationRecord'
    action_type = DurationRecorderAction
    goal = DurationRecorderGoal(
        destination='duration_recorder_test/',
        duration=rospy.Duration.from_sec(record_time),
        topics_to_record=[] # Empty records all topics, or provide a list of topics e.g. ['/rosout']
    )
    print('DurationRecorderGoal:')
else:
    print('Invalid recorder type. Please choose "rolling_recorder" or "duration_recorder"')
    sys.exit(-1)
print(goal)

def print_feedback(feedback_msg):
    print('Feedback:', feedback_msg.status)

rospy.init_node(NODE_NAME, log_level=rospy.DEBUG)
action_client = actionlib.SimpleActionClient(action, action_type)
res = action_client.wait_for_server()
action_client.send_goal(goal, feedback_cb=print_feedback)
action_client.wait_for_result(rospy.Duration.from_sec(record_time+5))

print('Goal state:', action_client.get_state())
print('Goal status text:', action_client.get_goal_status_text())
print('Goal', action_client.get_result())
