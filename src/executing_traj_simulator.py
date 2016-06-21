#! /usr/bin/env python

import roslib; roslib.load_manifest('behavioural_state_machine')
import rospy
from behavioural_state_machine.msg import trajectory

#Se for sim
/vizzy/neck_tilt_position_controller/command
/vizzy/eyes_tilt_position_controller/command

#Se for nao
/vizzy/neck_pan_position_controller/command
/vizzy/version_position_controller/command


