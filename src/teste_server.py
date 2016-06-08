#! /usr/bin/env python

import roslib; roslib.load_manifest('behavioural_state_machine')
import rospy
import actionlib
import behavioural_state_machine.msg
import numpy as np # to work with numerical data efficiently
import matplotlib.pyplot as plt





fs = 100 # sample rate 
f = 3 # the frequency of the signal




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
neck_yes_amplitude = neck_tilt_max - neck_tilt_min
neck_no_amplitude = neck_pan_max - neck_pan_max

#eyes limits (degrees)
eyes_max=38
eyes_min=-38
eyes_amplitude = eyes_max - eyes_min












class TesteAction(object):
  # create messages that are used to publish feedback/result
  _feedback = behavioural_state_machine.msg.TesteFeedback()
  _result   = behavioural_state_machine.msg.TesteResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name,
                                            behavioural_state_machine.msg.TesteAction, 
                                            execute_cb=self.execute_cb, auto_start = False)
    self._as.start()

  def execute_cb(self, goal):
    # helper variables
    r = rospy.Rate(1)
    success = True
    self._feedback.feed = 0
    
    # publish info to the console for the user
    print('goal.movement: {}'.format(goal.movement))


    self._feedback.feed = 1;      


    if goal.movement == 'y':

      x = np.arange(fs) # the points on the x axis for plotting
      # compute the value (amplitude) of the sin wave at the for each sample
      y = np.sin(2*np.pi*f * x / fs)*(neck_yes_amplitude/2) + neck_tilt_joint#position

      for i in range(len(y)):
        if y[i] > neck_tilt_max:
          y[i] = neck_tilt_max
        elif y[i] < neck_tilt_min:
          y[i] = neck_tilt_min

    elif goal.movement == 'n':

      x = np.arange(fs) # the points on the x axis for plotting
      # compute the value (amplitude) of the sin wave at the for each sample
      y = np.sin(2*np.pi*f * x / fs)*(neck_no_amplitude/2) + neck_pan_joint#position

      for i in range(len(y)):
        if y[i] > neck_pan_max:
          y[i] = neck_pan_max
        elif y[i] < neck_pan_min:
          y[i] = neck_pan_min









    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self._as.set_preempted()
      success = False

    if success:
      self._result.x = x
      self._result.y = y
      self._result.z = -y
      print('%s: succeeded' % self._action_name)
      plt.plot(self._result.x, self._result.y)
      plt.plot(self._result.x, self._result.z)
      plt.xlabel('sample(n)')
      plt.ylabel('angle')
      plt.show()
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('teste')
  TesteAction(rospy.get_name())
  rospy.spin()
