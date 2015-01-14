#!/usr/bin/env python
'''
Created on Nov 27, 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''

import rospy
import tf
from threading import Thread
from threading import Lock
    
class TfBroadcasterThread(Thread):
    def __init__(self,child_frame,parent_frame,tf_br=None):
        Thread.__init__(self)
        rospy.loginfo("Initializing tf broadcaster with child frame "+child_frame+" and parent frame "+parent_frame)
        if tf_br is None:
            self.tf_br = tf.TransformBroadcaster()
        else:
            self.tf_br = tf_br
        self.translation = None
        self.quaternion = None
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.has_transformation=False
        self.lock_=Lock()
            
    def set_transformation(self,translation,quaternion):
        self.lock_.acquire()
        self.translation = translation
        self.quaternion = quaternion
        self.lock_.release()
        self.has_transformation =True

    def run(self):
        r = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            try:
                if self.has_transformation:
                    if self.lock_.acquire(False):
                        self.tf_br.sendTransform(self.translation ,self.quaternion , rospy.Time.now(), self.child_frame,self.parent_frame)
                        self.lock_.release()
                
            except Exception,e:
                print 'TfBroadcasterThread:',e
            r.sleep()
        


