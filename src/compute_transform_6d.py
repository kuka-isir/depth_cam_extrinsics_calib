#!/usr/bin/env python
'''
Created on Nov 27, 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
'''

import rospy
import tf
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PointStamped
from threading import Thread
import numpy as np 
import sys
import argparse
import textwrap
from tf.transformations import quaternion_from_matrix
import message_filters
from message_filters import TimeSynchronizer
import itertools
import time
from threading import Lock

class ApproximateTimeSynchronizer(TimeSynchronizer):

    """
    Approximately synchronizes messages by their timestamps.
    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. The API is the same as TimeSynchronizer
    except for an extra `slop` parameter in the constructor that defines the delay (in seconds)
    with which messages can be synchronized
    """

    def __init__(self, fs, queue_size, slop):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = rospy.Duration.from_sec(slop)

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        for vv in itertools.product(*[list(q.keys()) for q in self.queues]):
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t]
        self.lock.release()
        
def rigid_transform_3D(A, B):
    assert len(A) == len(B)
    N = A.shape[0]; # total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))
    # dot is matrix multiplication for array
    H = np.transpose(AA) * BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T * U.T
    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = Vt.T * U.T
    t = -R*centroid_A.T + centroid_B.T
    return R, t

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
        self.lock=Lock()
            
    def set_transformation(self,translation,quaternion):
        self.lock.acquire()
        self.translation = translation
        self.quaternion = quaternion
        self.lock.release()
        self.has_transformation =True

    def run(self):
        while not rospy.is_shutdown():
            try:
                if self.has_transformation:
                    self.lock.acquire()
                    self.tf_br.sendTransform(self.translation ,self.quaternion , rospy.Time.now(), self.child_frame,self.parent_frame)
                    self.lock.release()
            except Exception,e:
                print 'TfBroadcasterThread:',e
             
class Estimator:
    def __init__(self,topic_a,topic_b,child_frame,parent_frame,transform_name="calib_kinect",dist_min_between_pts=0.03,n_pts_to_start_calib=3):
        rospy.init_node("compute_transformation_6d")
        self.rate = rospy.Rate(100)
        self.topic_a = topic_a
        self.topic_b = topic_b
        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.pts_a = []
        self.pts_b = []
        self.init_tf_broadcaster()
        self.static_transform = []
        self.init_subscribers()
        self.transform_name = transform_name
        self.MIN_NUM_CALIB = n_pts_to_start_calib
        self.min_d = dist_min_between_pts
        self.lock=Lock()
        self.cloud_a_pub = rospy.Publisher("/pointsA",PointCloud)
        self.cloud_b_pub = rospy.Publisher("/pointsB",PointCloud)
        
    def init_subscribers(self):
        rospy.loginfo("Initializing subscribers")
        sub=[]
        sub.append(message_filters.Subscriber(self.topic_a, PointStamped))
        sub.append(message_filters.Subscriber(self.topic_b, PointStamped))
        ts = ApproximateTimeSynchronizer(sub, 2,0.05)
        #ts.registerCallback(self.callback)        
        #ts = message_filters.TimeSynchronizer(sub, 20)
        ts.registerCallback(self.callback)
        
    def init_tf_broadcaster(self):        
        rospy.loginfo("Initializing the Tf broadcaster")
        self.tf = tf.TransformListener()
        self.tf_thread = TfBroadcasterThread(self.parent_frame,self.child_frame)
    
    def publish_pointcloud(self,cloud,frame_out,cloud_publisher):
        cloud_out = PointCloud()
        cloud_out.header.frame_id = frame_out
        cloud_out.header.stamp = rospy.Time.now()
        for p in cloud:
            p_out = Point32()
            p_out.x = p.item(0)
            p_out.y = p.item(1)
            p_out.z = p.item(2)
            cloud_out.points.append(p_out)
        cloud_publisher.publish(cloud_out)
              
    def is_point_far_enough(self,p_in,list_of_pt,dmin):
        for p in list_of_pt:
            p1 = np.array(p)
            p2 = np.array(p_in)
            vd = p1-p2
            d = np.linalg.norm(vd)
            if d<dmin:
                return False
        return True
            
    def callback(self,*msg):
        add_new_pt = True
        pta = [msg[0].point.x ,msg[0].point.y ,msg[0].point.z]
        ptb = [msg[1].point.x ,msg[1].point.y ,msg[1].point.z]
        
        self.lock.acquire()
        if(len(self.pts_a)>=self.MIN_NUM_CALIB):
            add_new_pt = self.is_point_far_enough(pta,self.pts_a,self.min_d)
        self.lock.release()
        #if len(self.pts_a)>0 and np.linalg.norm([self.pts_a[-1],pta]) < self.min_d:
        #    rospy.logwarn("Point to close, not adding ",pta," d=",np.linalg.norm([self.pts_a[-1],pta]),"dmin=",self.min_d)
        #    add_new_pt = False
        if add_new_pt:
            self.lock.acquire()
            self.pts_a.append(pta)
            self.pts_b.append(ptb)
            self.lock.release()
        #time.sleep(2.0)
        
    def start(self):
        self.tf_thread.start()
        while not rospy.is_shutdown():
            if len(self.pts_a)>self.MIN_NUM_CALIB:
                self.lock.acquire()
                A = np.matrix(self.pts_a)
                B = np.matrix(self.pts_b)
                self.lock.release()
    
    
                            
                self.publish_pointcloud(A,self.child_frame,self.cloud_a_pub)
                self.publish_pointcloud(B,self.parent_frame,self.cloud_b_pub)
                #print "Points A"
                #print A
                #print ""
                
                #print "Points B"
                #print B
                #print ""
                
                ret_R, ret_t = rigid_transform_3D(B, A)
                new_col = ret_t.reshape(3, 1)
                tmp = np.append(ret_R, new_col, axis=1)
                aug=np.array([[0.0,0.0,0.0,1.0]])
                translation = np.squeeze(np.asarray(ret_t))
                T = np.append(tmp,aug,axis=0)
                quaternion = quaternion_from_matrix(T)
                
                self.tf_thread.set_transformation(ret_t,quaternion)
                self.static_transform = '<node pkg="tf" type="static_transform_publisher" name="'+self.transform_name+'" args="'\
                +' '.join(map(str, translation))+' '+' '.join(map(str, quaternion))+' '+self.child_frame+' '+self.parent_frame+' 100" />'
                
                #print self.static_transform
    
                print "Translation - Rotation"
                print translation,quaternion                

            else:
                rospy.loginfo("Waiting for %d new points to start calibration."%(self.MIN_NUM_CALIB-len(self.pts_a)))
                time.sleep(1.0)
            self.rate.sleep()
            
        
def main(argv):
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,description=textwrap.dedent(""" Estimates the best 6D transformation between two set of 3D points"""),epilog='Maintainer: Antoine Hoarau <hoarau.robotics AT gmail DOT com>')
    parser.add_argument('topic_a', type=str,help='First topic to listen (geometry_msgs::PointStamped)',default='/tracker/ball_position')
    parser.add_argument('topic_b', type=str,help='Second topic to listen (geometry_msgs::PointStamped)  ',default='/kuka/tooltip_position')    
    parser.add_argument('child_frame', type=str,help='Child Frame (should match first topic)',default='/camera_depth_optical_frame')
    parser.add_argument('parent_frame', type=str,help='Second Frame (should match second topic)',default='/base_link')
    parser.add_argument('--name', type=str,help='Transformation name (for roslaunch)',default='calib_kinect')
    parser.add_argument('-d','--dmin_between_pts', type=str,help='Distance min to get a new point',default=0.03)
    parser.add_argument('-m','--min_pts_to_start', type=str,help='Min number of points before we start calibrating',default=3)
    args,_ = parser.parse_known_args()
    E = Estimator(args.topic_a,args.topic_b,args.child_frame,args.parent_frame,args.name,args.dmin_between_pts,args.min_pts_to_start)
    E.start()
    
if __name__ == '__main__':
    main(sys.argv)
    exit(0)
    

