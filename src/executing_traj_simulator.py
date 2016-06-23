#! /usr/bin/env python

import roslib; roslib.load_manifest('behavioural_state_machine')
import rospy
from behavioural_state_machine.msg import trajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time


def cb_once(msg):

	print 'executing...'

	if msg.mov == 'y':
		for i in range(len(msg.neck)):
			pub1.publish(msg.neck[i])
			pub2.publish(-msg.eyes[i])
			print 'executing yes...'
			time.sleep(0.1) 
	if msg.mov == 'n':
		for i in range(len(msg.neck)):
			pub3.publish(msg.neck[i])
			pub4.publish(msg.eyes[i])
			print 'executing no...'
			time.sleep(0.1)

def main():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener_traj', anonymous=True)

	global pub1
	global pub2
	global pub3
	global pub4
	pub1 = rospy.Publisher('/vizzy/neck_tilt_position_controller/command', Float64, queue_size=10)
	pub2 = rospy.Publisher('/vizzy/eyes_tilt_position_controller/command', Float64, queue_size=10)
	pub3 = rospy.Publisher('/vizzy/neck_pan_position_controller/command', Float64, queue_size=10)
	pub4 = rospy.Publisher('/vizzy/version_position_controller/command', Float64, queue_size=10)



	rospy.Subscriber('traj', trajectory, cb_once)
	#rospy.wait_for_message('traj', JointState)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()



#Se for sim
#/vizzy/neck_tilt_position_controller/command
#/vizzy/eyes_tilt_position_controller/command

#Se for nao
#/vizzy/neck_pan_position_controller/command
#/vizzy/version_position_controller/command


if __name__ == '__main__':
	main()