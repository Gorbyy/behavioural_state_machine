#! /usr/bin/env python

import numpy as np # to work with numerical data efficiently
import matplotlib.pyplot as plt
import rospy
import sensor_msgs.msg

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




try:
    cena = rospy.wait_for_message('/vizzy/joint_states', sensor_msgs.msg.JointState, timeout=10)
    print cena
except(rospy.ROSException), e:
    print "Laser scan topic not available, aborting..."
    print "Error message: ", e
    exit()