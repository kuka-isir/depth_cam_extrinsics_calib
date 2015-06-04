#!/usr/bin/python
'''
Created on Mar 25, 2013

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''
import subprocess
import rospy
from sensor_msgs.msg import Image
import message_filters
import cv2
from cv_bridge import CvBridge, CvBridgeError
import threading
from threading import Thread,Lock
import os
import numpy as np

class CompressTopic(Thread):
    def __init__(self,topic,caller_id,compress_mode = 'compressed'):
        Thread.__init__(self)
        assert not isinstance(topic, list)
        self.topic_in = topic
        self.topic_out = topic+'_comp'+caller_id
        self.compress_mode = compress_mode


    def run(self):
        subprocess.call('rosrun image_transport republish ' +self.compress_mode+' in:='+self.topic_in+' out:='+self.topic_out,shell=True)
        print self,'stopped'

class ROSImageSubscriber(Thread):
    '''
    Generic tool for ros images
    '''
    def __init__(self,topics,queue_size=1,use_compression=True,loop_rate = 1.0):
        Thread.__init__(self)
        self.node_name = 'img_subscriber'
        try:
            rospy.init_node(self.node_name, anonymous=True)
        except: pass
        self.caller_id = rospy.get_name()
        self.caller_id  = self.caller_id[len(self.node_name)+1:]
        if not isinstance(topics, list):
            topics = [topics]
        self.topics = topics
        self.pid = os.getpid()
        #self.daemon = False
        if(use_compression):
            self.start_compression_threads()

        print(str(self)+' subscribing to : '+str(self.topics))
        self.mutex = [Lock()]*len(topics)
        
        self.has_received_first = [False]*len(topics)
        self.should_register_mouse = [False]*len(topics)
        self.mouse_function = [self.wtf]*len(topics)
        self.bridge = CvBridge()
        self.images=[]
        self._stop = threading.Event()
        self.lock_ = Lock()
        self.loopy = rospy.Rate(loop_rate)
        if len(self.topics) == 1:
            rospy.Subscriber(self.topics[0], Image,self.callback,queue_size=queue_size, buff_size=2**24)
            self.images.append(np.array([]))
        else:
            sub=[]
            for topic in self.topics:
                sub.append(message_filters.Subscriber(topic, Image))
                self.images.append(np.array([]))
            ts = message_filters.TimeSynchronizer(sub, queue_size)
            ts.registerCallback(self.callback)
            
    def lock(self):
        self.lock_.acquire()
        
    def release(self):
        self.lock_.release()
        
    def locked(self):
        return self.lock_.locked()
        
    def start_compression_threads(self):
        self.comp=[]
        for i in xrange(0,len(self.topics)):
            isdepth = self.topics[i].find('/depth') >=0
            if not isdepth:
                self.comp.append(CompressTopic(self.topics[i],self.caller_id,compress_mode='compressed'))
            else:
                self.comp.append(CompressTopic(self.topics[i],self.caller_id,compress_mode='compressedDepth'))
        for i in xrange(0,len(self.topics)):
            self.topics[i] = self.comp[i].topic_out
            self.comp[i].start()
    
    def register_mouse_callback(self,function):
        for i in xrange(len(self.topics)):
            self.mouse_function[i] = function
            self.should_register_mouse[i] = True

    def mouse_callback_spin_once(self):
        for i in xrange(len(self.topics)):
            if self.should_register_mouse[i]:
                window = self.topics[i]+str(i)
                function = self.mouse_function[i]
                print 'Registering',function,'for window name',window
                cv2.setMouseCallback(window,function)
                self.should_register_mouse[i] = False

    def get_window_name(self):
        if len(self.topics)==1:
            window = self.topics[0]+str(0)
        else:
            window = []
            for i in xrange(len(self.topics)):
                window.append(self.topics[i]+str(i))
        return window

    def stop(self):

        self._stop.set()       
        print self,' stopped'

    def wtf(self):
        for i in xrange(0,len(self.topics)):
            if self.has_received_first[i]:
                print 'I have received at least 1 image from : ',self.topics[i]
            else:
                print "I'm still waiting for : ",self.topics[i]
    def run(self):
        rospy.spin()
        try:
            while not rospy.is_shutdown():
                self.loopy.sleep()

        except KeyboardInterrupt:
            self.stop()
        for i in xrange(len(self.images)):
            self.mutex[i].acquire()
            #cv2.destroyAllWindows()#(self.topics[i]+str(i))
            cv2.destroyWindow(self.topics[i]+str(i))
            self.mutex[i].release()
            
    def callback(self,*msg):
        if self.locked():
            #print "locked"
            return 
        for i in xrange(len(self.topics)):
            try:
                enc = msg[i].encoding
                if enc=='rgb8':
                    # Strange kinect exception
                    enc='bgr8'
                    
                if not self.has_received_first[i]:
                    self.has_received_first[i] = True

                self.mutex[i].acquire()
                try:
                    self.images[i] =  self.bridge.imgmsg_to_cv2(msg[i],enc)
                    
                except Exception,e:
                    print "Warning, using old cv brdige : ",e
                    self.images[i] =  np.array(self.bridge.imgmsg_to_cv(msg[i],enc))
                self.mutex[i].release()
            except CvBridgeError, e:
                print e

    def get_image(self,i=0,blocking=True):
        if not blocking:   
            
            while not self.mutex[i].acquire(False):
                pass
            img = self.images[i]
            self.mutex[i].release()
            return img
        else:
            if self.mutex[i].acquire(False):
                img = self.images[i]
                self.mutex[i].release()
                return img
        return None
        
    def show_image(self,window_name,image):
        if image is None:
            return
        if not rospy.is_shutdown(): 
            try:
                isdepth = window_name.find('/depth') >=0
                if isdepth:
                    depth_array = np.array(image, dtype=np.float32)
                    # Normalize the depth image to fall between 0 (black) and 1 (white)
                    cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
                    cv2.imshow(window_name, depth_array)
                isir = window_name.find('/ir') >=0
                if isir:
                    ir_array = np.array(image, dtype=np.float32)
                    # Normalize the depth image to fall between 0 (black) and 1 (white)
                    cv2.normalize(ir_array, ir_array, 0, 1, cv2.NORM_MINMAX)
                    cv2.imshow(window_name, ir_array)
                else:
                    cv2.imshow(window_name, image)
                self.mouse_callback_spin_once()
            except TypeError,e:
                print e

    def show(self):
        for i in xrange(len(self.topics)): 
            if self.has_received_first[i]:
                    img = self.get_image(i,blocking=False)
                    if img is not None:
                        self.show_image(self.topics[i]+str(i),img )
                        cv2.waitKey(3)
