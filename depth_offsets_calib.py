#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  9 13:15:12 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from ros_image_tools.kinect_v2 import Kinect_v2
from ros_image_tools.kinect import Kinect
import rospy
import time
import cv2
import tf
import sys
import numpy as np
from threading import Thread
from scipy import stats

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
    def __init__(self,kinect_type, kinect_name,base_frame,serial,output_file=None):
        Thread.__init__(self)
        if kinect_name[-1] == '/':
            kinect_name = kinect_name[:-1]
        self.output_file_path = output_file
        
        if (kinect_type == "Kinect2") or (kinect_type == "Kinectv2") or (kinect_type == "Kinect_v2"):
            print "Loading Kinect2 with serial : "+serial 
            self.kinect = Kinect_v2(kinect_name,serial,queue_size=10,compression=False,use_rect=True,use_ir=False)
        elif kinect_type == "Kinect":
            print "Loading Kinect1 with serial : "+serial
            self.kinect = Kinect(kinect_name,queue_size=10,compression=False,use_rect=True,use_depth_registered=True,use_ir=False)
        else:
            print "ERROR: Kinect type must be Kinect2 or Kinect"
            return       
        
        self.kinect_name = kinect_name
        self.base_frame = base_frame
        self.transform_name = 'calib_'+self.kinect_name[1:]
        self.kinect.wait_until_ready()

        self.rgb_values = []
        self.depth_values = []
            
    def start(self):

        listener = tf.TransformListener()
        # TODO make parameters
        while not rospy.is_shutdown():
            try:
                (trans,rot) = listener.lookupTransform('/kinect3_rgb_optical_frame', '/ar_marker_0', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue 
            
            (pixel_x, pixel_y) = self.kinect.world_to_depth(trans,use_distortion=False)
            pixel_x = int(pixel_x)
            pixel_y = int(pixel_y)
            self.kinect.show_rgb()
            
            rgb_img = self.kinect.get_rgb(blocking=False)
            cv2.circle(rgb_img, (pixel_x,pixel_y), 6, (0,0,255), -1)
            depth_tag_frame = self.kinect.depth_to_world(pixel_x, pixel_y,transform_to_camera_link=False)

            if not (True in np.isnan(depth_tag_frame)) and (depth_tag_frame is not None):          
                depth_tag_frame = depth_tag_frame/1000                

                dist_rgb = np.linalg.norm(np.array(trans))
                dist_depth = np.linalg.norm(depth_tag_frame)
                
                if dist_rgb>0.5:
                    self.rgb_values.append(dist_rgb)
                    self.depth_values.append(dist_depth)
                    
                    slope, intercept, r_value, p_value, std_err = stats.linregress(self.depth_values,self.rgb_values)
                    print 'slope: ', slope
                    print 'z0', intercept,'\n'
                    time.sleep(1.0)
                    
                    if len(self.depth_values)>15:
                        return
                        
def main(argv):
    rospy.init_node("depth_offsets_calib",anonymous=True)
    if rospy.has_param("/use_sim_time"):
        rospy.logwarn("Using simulation time")
        while not rospy.Time.now():
            pass # tsim syncing
       
    kinect_name = rospy.get_param('~camera_name')
    base_frame = rospy.get_param('~base_frame')
    output_file = rospy.get_param('~output_file')
    serial = rospy.get_param('~serial')
    kinect_type = rospy.get_param('~kinect_type')
    
    calib = KinectDepthOffsetsCalibration(kinect_type, kinect_name, base_frame, serial, output_file)
    calib.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)
