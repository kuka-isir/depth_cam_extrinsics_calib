#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  9 13:15:12 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from depth_cam_tools.kinect1 import Kinect1
from depth_cam_tools.xtion_pro_live import XtionProLive
import rospy
import cv2
import sys
import numpy as np
from threading import Thread
from scipy import stats
from ar_track_alvar_msgs.msg import AlvarMarkers

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is True for "yes" or False for "no".
    """
    valid = {"yes": True, "y": True, "ye": True,
             "no": False, "n": False}
    if default is None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "
                             "(or 'y' or 'n').\n")

class KinectDepthOffsetsCalibration(Thread):
    def __init__(self,kinect_type, kinect_name, marker_id, nb_pts, dz, output_file):
        Thread.__init__(self)
        if kinect_name[-1] == '/':
            kinect_name = kinect_name[:-1]

        if (kinect_type == "Kinect2") or (kinect_type == "Kinectv2") or (kinect_type == "KinectV2"):
            self.kinect_type = "Kinect2"
            print "Loading "+kinect_name+" of type Kinect2"
            print "ERROR: Kinect2 offset is already been computed during the intrinsics calibration. Nothing to be done here"
        elif (kinect_type == "Kinect") or (kinect_type == "Kinect1") or (kinect_type == "Kinectv1") or (kinect_type == "KinectV1"):
            self.kinect_type = "Kinect1"
            print "Loading "+kinect_name+" of type Kinect1"
            self.kinect = Kinect1(kinect_name,queue_size=10,compression=False,use_rect=True,use_depth_registered=True,use_ir=False)
        elif (kinect_type == "Xtion"):
            self.kinect_type = "Xtion"
            print "Loading "+kinect_name+" of type Xtion"
            self.kinect = XtionProLive(kinect_name,queue_size=10,compression=False,use_rect=True,use_depth_registered=True,use_ir=False)
        else:
            print "ERROR: Kinect type must be Kinect1 or Xtion"
            return 
        
        self.marker_id = marker_id
        self.output_file = output_file
        self.nb_pts = nb_pts
        self.dz = dz
        self.kinect_name = kinect_name
        self.transform_name = 'calib_'+self.kinect_name[1:]
        self.kinect.wait_until_ready()
        
        self.last_used_time = rospy.Time.now()
        self.rgb_values = []
        self.depth_values = []
        
        self.slope = 1.0
        self.intercept = 0
        
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers , self.callback)
        rospy.on_shutdown(self.save_params)
        
    def callback(self,data):
        self.kinect.show_rgb()

        if len(data.markers)>0:
            for i in range(len(data.markers)):
                if (self.marker_id==-1) or (self.marker_id==data.markers[i].id):
                    trans = [0,0,0]            
                    trans[0] = data.markers[0].pose.pose.position.x
                    trans[1] = data.markers[0].pose.pose.position.y
                    trans[2] = data.markers[0].pose.pose.position.z
                    
                    source_frame = data.markers[0].header.frame_id
                    target_frame = self.kinect_name+'_rgb_optical_frame'
                    
                    trans = self.kinect.transform_point(trans, target_frame , source_frame)
            
                    (pixel_x, pixel_y) = self.kinect.world_to_depth(trans,use_distortion=True)
                    pixel_x = int(pixel_x)
                    pixel_y = int(pixel_y)            
                    
                    rgb_img = self.kinect.get_rgb(blocking=False)
                    cv2.circle(rgb_img, (pixel_x,pixel_y), 6, (0,0,255), -1)
                    depth_tag_frame = self.kinect.depth_to_world(pixel_x, pixel_y,transform_to_camera_link=False)
                    cv2.imshow("RGB", rgb_img) 
                    
                    diff_time = (rospy.Time.now() - self.last_used_time).to_sec()
                    if diff_time>1.0:            
                        if not (True in np.isnan(depth_tag_frame)) and (depth_tag_frame is not None):               
                
                            dist_rgb = np.linalg.norm(np.array(trans))
                            dist_depth = np.linalg.norm(depth_tag_frame)
                            
                            min_d = 100.0                    
                            for rgb_i in self.rgb_values:
                                d = abs(rgb_i - dist_rgb)
                                if d<min_d:
                                    min_d=d
                            
                            if min_d<self.dz:
                                return
                            
                            if dist_rgb>0.51:
                                self.last_used_time = rospy.Time.now()
                                self.rgb_values.append(dist_rgb)
                                self.depth_values.append(dist_depth)
                                
                                self.slope, self.intercept, r_value, p_value, std_err = stats.linregress(self.depth_values,self.rgb_values)
                                print 'slope: ', self.slope
                                print 'z0', self.intercept
                                print 'slope error',std_err, '\n'
                                
                                if len(self.depth_values)>=self.nb_pts:
                                    rospy.signal_shutdown('CALIBRATION DONE !')
                                    
    def save_params(self):
        print ''
        if not query_yes_no("Do you want to save these parameters to the file "+self.output_file +" ?","no"):
            print "Exiting without saving params !"                        
        else:
            print 'Saving params slope=',self.slope,' and intercept=',int(self.intercept*1000),' to file: ', self.output_file
            try:
                with open(self.output_file,'r') as f:
                    with open(self.output_file+'.bak','w') as fbak:
                        print self.output_file,' already exists, creating backup file.'
                        fbak.write(f.read())
            except: pass
            with open(self.output_file,'w') as f:
                f.write(
"""z_offset_mm: """+str(int(self.intercept*1000))+
"""
z_scaling: """ +str(self.slope)
)
            print "File saved."             
                        
def main(argv):
    rospy.init_node("depth_offsets_calib",anonymous=True)
    if rospy.has_param("/use_sim_time"):
        rospy.logwarn("Using simulation time")
        while not rospy.Time.now():
            pass # tsim syncing
       
    kinect_name = rospy.get_param('~camera_name')
    kinect_type = rospy.get_param('~kinect_type')
    nb_pts = rospy.get_param('~nb_pts')
    dz = rospy.get_param('~dz')
    output_file = rospy.get_param('~output_file')
    marker_id = rospy.get_param('~marker_id')
    
    calib = KinectDepthOffsetsCalibration(kinect_type, kinect_name, marker_id, nb_pts, dz, output_file)
    calib.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)
