#! /usr/bin/env python

import roslib; roslib.load_manifest('behavioural_state_machine')
import rospy

import actionlib

import behavioural_state_machine.msg
 

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
    rospy.loginfo('action_name: %s, goal.limits: %i, feedback.feed: %i' % (self._action_name, goal.limits, self._feedback.feed))
    


    # start executing the action
    for i in xrange(1, goal.limits):
      # check that preempt has not been requested by the client
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        success = False
        break

      self._feedback.feed = i;
      print 'feedback: %i' %self._feedback.feed
      
      # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep()

    if success:
      self._result.res = self._feedback.feed
      rospy.loginfo('%s succeeded, result: %i' % (self._action_name, self._result.res))
      self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
  rospy.init_node('teste')
  TesteAction(rospy.get_name())
  rospy.spin()
