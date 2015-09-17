#!/usr/bin/env python
'''
Created on Nov 27, 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''

import rospy
import tf
from threading import Thread
from threading import Lock
from threading import Event
import time
class TfBroadcasterThread(Thread):
    def __init__(self,child_frame,parent_frame,tf_br=tf.TransformBroadcaster()):
        Thread.__init__(self)
        rospy.loginfo("Initializing tf broadcaster with child frame "+child_frame+" and parent frame "+parent_frame)
        self.tf_br = tf_br
        self.translation = None
        self.quaternion = None
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.has_transformation = Event()
        self.lock_= Lock()
            
    def set_transformation(self,translation,quaternion):
        self.lock_.acquire()
        self.translation = translation
        self.quaternion = quaternion
        self.lock_.release()
        self.has_transformation.set()

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.has_transformation.is_set() and self.lock_.acquire(False):
                        self.tf_br.sendTransform(self.translation ,self.quaternion , rospy.Time.now(), self.child_frame,self.parent_frame)
                        self.lock_.release()
                
            except Exception,e:
                print 'TfBroadcasterThread:',e
                raise KeyboardInterrupt
            time.sleep(0.01)
        


