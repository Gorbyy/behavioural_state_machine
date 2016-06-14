#! /usr/bin/env python

import numpy as np # to work with numerical data efficiently
import matplotlib.pyplot as plt
import rospy
import sensor_msgs.msg
from std_msgs.msg import String

amplitude = 200
limitmax = amplitude/2
limitmin = -amplitude/2

position = 50




fs = 500 # sample rate 
f = 3 # the frequency of the signal

x = np.arange(fs) # the points on the x axis for plotting
# compute the value (amplitude) of the sin wave at the for each sample
y = np.sin(2*np.pi*f * x / fs)*(amplitude/2) + position

for i in range(len(y)):
	if y[i] > limitmax:
		y[i] = limitmax
	elif y[i] < limitmin:
		y[i] = limitmin


plt.plot(x, y)
plt.plot(x, -y)
plt.xlabel('sample(n)')
plt.ylabel('angle')
plt.show()


#index on the msg joint_states
eyes_tilt_joint = 2
neck_pan_joint = 17
neck_tilt_joint = 18
version_joint = 32

#neck limits (degrees)
neck_pan_max = 53
neck_pan_min = -53
neck_tilt_max = 37
neck_tilt_min = -18

#eyes limits (degrees)
eye_max=38
eye_min=-38


#try:
#    cena = rospy.wait_for_message('/vizzy/joint_states', sensor_msgs.msg.JointState, timeout=10)
#    print cena
#except(rospy.ROSException), e:
#    print "Laser scan topic not available, aborting..."
#    print "Error message: ", e



def cb_once(msg):
	 rospy.loginfo(rospy.get_caller_id() + 'I heard %s', msg.data)


def main():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber('chat', String, cb_once)
	#msg = rospy.wait_for_message('chat', String)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	main()
