#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 03 15:47:00 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from ros_image_tools.kinect import Kinect
from ros_image_tools.tf_broadcaster import TfBroadcasterThread
import rospy
import time
import cv2
from math import pi
import argparse,textwrap,sys
from geometry_msgs.msg import PointStamped
import numpy as np
from threading import Lock
from tf.transformations import quaternion_from_matrix
from threading import Thread,Event
from sensor_msgs.msg import PointCloud
import scipy.spatial.distance as scipy_dist


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

class KinectSinglePointsCalibrationExtrinsics(Thread):
    def __init__(self,kinect_name,base_frame,output_file=None):
        Thread.__init__(self)
        if kinect_name[-1] == '/':
            kinect_name = kinect_name[:-1]
        self.output_file_path = output_file
        self.kinect = Kinect(kinect_name,queue_size=10,compression=False,use_rect=True,use_depth_registered=True,use_ir=True)
        self.kinect_name = kinect_name
        self.base_frame = base_frame
        self.transform_name = 'calib_'+self.kinect_name[1:]
        self.kinect.wait_until_ready()
        self.kinect.register_mouse_callbacks(self.mouse_callback)
        
        self.depth_pt_pub = rospy.Publisher(self.kinect_name+'/calibration/pts_depth',PointCloud,queue_size=10)
        self.world_pt_pub = rospy.Publisher(self.kinect_name+'/calibration/pts_calib',PointCloud,queue_size=10)
        self.pt_pub = rospy.Publisher(self.kinect_name+'pts_detected',PointStamped,queue_size=10)

        self.A=[]
        self.B=[]         
        self.pt2d=[]
        self.pt2d_fit=[]
        self.single_pt_pos=[]
        
        self.lock_=Lock()
        self.event_ = Event()
        self.mouse_clicked = False
        
        self.tf_thread = TfBroadcasterThread(self.kinect.link_frame,self.base_frame)
        
    def mouse_callback(self,event,x,y,flags,param):
        if self.lock_.locked() or self.event_.is_set():
            print "locked at ",rospy.Time.now()
            return
        
        if event == cv2.EVENT_RBUTTONUP:
            self.event_.set()
            self.mouse_clicked = True
            self.event_.clear()
#==============================================================================
#         if event == cv2.EVENT_MOUSEMOVE:
#              ir = np.array(self.kinect.get_ir(blocking=False))
#              xyz = self.kinect.depth_to_world(x,y)
#              cv2.putText(ir,"x y z : "+str(xyz), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250,250,255))
#             
#==============================================================================
            
    def calibrate3d(self):
        #self.lock_.acquire()
        print self.A
        A = np.matrix(self.A)
        B = np.matrix(self.B)
        

        ret_R, ret_t = rigid_transform_3D(A, B)

        new_col = ret_t.reshape(3, 1)
        tmp = np.append(ret_R, new_col, axis=1)
        aug=np.array([[0.0,0.0,0.0,1.0]])
        translation = np.squeeze(np.asarray(ret_t))
        T = np.append(tmp,aug,axis=0)
        quaternion = quaternion_from_matrix(T)

        print "Translation - Rotation"
        print translation,quaternion
        # Send the transform to ROS
        self.tf_thread.set_transformation(ret_t,quaternion)


        invR = ret_R.T
        invT = -invR * ret_t

        ## Compute inverse of transformation
        B_in_A = np.empty(B.shape)
        for i in xrange(len(B)):
            p = invR*B[i].T + invT
            B_in_A[i] = p.T

        ## Compute the standard deviation
        err = A-B_in_A
        std = np.std(err,axis=0)
        print "Standard deviation : ",std


#==============================================================================
#         self.pt2d_fit = []
#         for p_orig2d,p in zip(self.pt2d,B_in_A): #chess dans /camera_link
#             #print "p_orig2d:",p_orig2d
#             pdepth = self.kinect.transform_point(p,self.kinect.depth_optical_frame,self.kinect.link_frame)
#             #print "pdepth:",pdepth
#             pfinal = self.kinect.world_to_depth(pdepth)
#             #print "pfinal:",pfinal
#             self.pt2d_fit.append(pfinal)
# 
#         self.depth_pt_pub.publish(self.get_prepared_pointcloud(A,self.kinect.link_frame))
#         self.world_pt_pub.publish(self.get_prepared_pointcloud(B,self.base_frame))
#         print ""
#         self.static_transform = '<node pkg="tf" type="static_transform_publisher" name="'+self.transform_name+'" args="'\
#         +' '.join(map(str, translation))+' '+' '.join(map(str, quaternion))+' '+self.base_frame+' '+self.kinect.link_frame+' 100" />'
#         print self.static_transform
#         print ""
#==============================================================================
        #self.lock_.release()

    def get_prepared_pointcloud(self,pts,frame):
        cloud = PointCloud()
        cloud.header.frame_id=frame
        cloud.header.stamp = rospy.Time.now()
        for p in pts:
            cloud.points.append(self.get_point_stamped(p,frame).point)
        return cloud

    def get_point_stamped(self,pt,frame):
        pt_out = PointStamped()
        pt_out.header.frame_id=frame
        pt_out.header.stamp = rospy.Time.now()
        if type(pt) == np.matrixlib.defmatrix.matrix:
            pt = pt.tolist()[0]
        pt_out.point.x = pt[0]
        pt_out.point.y = pt[1]
        pt_out.point.z = pt[2]
        return pt_out       

    def save_calibration(self):
        #TODO
#==============================================================================
#         if not self.static_transform or not self.output_file_path:
#             print 'Not saving files'
#             return
#         if query_yes_no("Do you want to save "+str(self.output_file_path)):
#             print "Saving file ",self.output_file_path
#             try:
#                 with open(self.output_file_path,'r') as f:
#                     with open(self.output_file_path+'.bak','w') as fbak:
#                         print self.output_file_path,' already exists, creating backup file.'
#                         fbak.write(f.read())
#             except: pass
#             with open(self.output_file_path,'w') as f:
#                 print self.static_transform
#                 f.write("""
# <launch>
#    """+self.static_transform+
# """
# </launch>
# """)
#             print "File saved."
#         else:
#             print "Not saving calibration."
#==============================================================================
        return    
                    
    def nothing(x,c):
        pass
            
    def start(self):
        
        cv2.namedWindow('Thresholding')
        cv2.createTrackbar('Threshold','Thresholding',160,255,self.nothing)
        cv2.namedWindow('DepthAreaSelection')
        cv2.createTrackbar('min','DepthAreaSelection',0,255,self.nothing) 
        cv2.createTrackbar('max','DepthAreaSelection',255,255,self.nothing) 
        
        while not rospy.is_shutdown():

            self.kinect.show_ir()
                        
            ir_array = np.array(self.kinect.get_ir(blocking=False), dtype=np.float32)
            cv2.normalize(ir_array, ir_array, 0, 1, cv2.NORM_MINMAX)
            depth_array = np.array(self.kinect.get_depth(blocking=False), dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

            ir_8u = ir_array*255
            ir_8u = ir_8u.astype(np.uint8)
            depth_8u = depth_array*255
            depth_8u = depth_8u.astype(np.uint8)
            cv2.imshow("depth", depth_8u)
            
#==============================================================================
#             min_dist = cv2.getTrackbarPos('min','DepthAreaSelection')
#             max_dist = cv2.getTrackbarPos('max','DepthAreaSelection')
#             depth_8u_mask = (depth_8u>min_dist)*(depth_8u<max_dist)
#             depth_8u_masked = depth_8u*depth_8u_mask
#             cv2.imshow("DepthAreaSelection", depth_8u_masked)      
#==============================================================================
            
            ir_8u = cv2.GaussianBlur(ir_8u ,(5,5),3)
            cv2.imshow("Gaussian Blur", ir_8u)              
            
            #ir_8u = ir_8u*depth_8u_mask[:,:,0]
            #cv2.imshow("Mask applied", ir_8u)                     
            
            thresh = cv2.getTrackbarPos('Threshold','Thresholding')
            ret, ir_8u_thresh = cv2.threshold(ir_8u,thresh,255,cv2.THRESH_BINARY)
            cv2.imshow("Thresholding", ir_8u_thresh) 

            kernel = np.ones((3,3),np.uint8)
            opening = cv2.morphologyEx(ir_8u_thresh, cv2.MORPH_OPEN, kernel)
            cv2.imshow("Opening", opening)                    
         
            if self.mouse_clicked:
                self.mouse_clicked = False
                
                opening_color = cv2.cvtColor(opening ,cv2.COLOR_GRAY2RGB)
                opening_color *= 255                

                # If there is the right amount of contours
                contours, _ = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
                nb_contours = len(contours)                         
                if nb_contours == 2:
                    pt = []
                    middle_x = 0
                    middle_y = 0
                    for i in range(nb_contours):
                        img = np.zeros((480,640,1), np.uint8)
                    
                        (x,y), radius = cv2.minEnclosingCircle(contours[i])
                        (x,y) = (int(x),int(y))
                        radius = int(radius)

                        cv2.circle(img, (x,y), radius, 255, -1)
                        cv2.imshow("DrawContours", img)
                        cv2.waitKey(10)
                        middle_x += x/nb_contours
                        middle_y += y/nb_contours
                    
                    middle_x = int(middle_x)
                    middle_y = int(middle_y)
                    cv2.circle(opening_color,(middle_x,middle_y),2,(0,255,0),-1)  
                    
                    pt = self.kinect.depth_to_world(middle_x,middle_y)
                    if not (True in np.isnan(pt)):
                        print ' => [',pt[0],pt[1],pt[2],']'
                        #pt.append(pt_temp)
                        pt_stamped = PointStamped()
                        pt_stamped.header.frame_id = rospy.get_param('~camera_frame')
                        pt_stamped.point.x = pt[0]/1000
                        pt_stamped.point.y = pt[1]/1000
                        pt_stamped.point.z = pt[2]/1000
                        self.pt_pub.publish(pt_stamped)
                    
                    print "detection"                
                
#==============================================================================
#                 pt=[]
#                 for i in range(len(contours)):
#                     img = np.zeros((480,640,1), np.uint8)
#                     
#                     (x,y), radius = cv2.minEnclosingCircle(contours[i])
#                     (x,y) = (int(x),int(y))
#                     radius = int(radius)
# 
#                     cv2.circle(img, (x,y), radius, 255, -1)
#                     cv2.imshow("DrawContours", img)
#                     cv2.waitKey(10)
#                     
#                     pt_temp = self.kinect.depth_to_world(x,y)  #Add depth shift because of markers' radius ??
#                     
#                     
#                     cv2.circle(opening_color,(x,y),radius,(0,255,0))  
#                                         
#                     if not (True in np.isnan(pt_temp)):
#                         print ' => [',pt_temp[0],pt_temp[1],pt_temp[2],']'
#                         pt.append(pt_temp)
#                         pt_stamped = PointStamped()
#                         pt_stamped.header.frame_id = "camera_link"
#                         pt_stamped.point.x = pt_temp[0]/1000
#                         pt_stamped.point.y = pt_temp[1]/1000
#                         pt_stamped.point.z = pt_temp[2]/1000
#                         self.pt_pub.publish(pt_stamped)
#                     
#                 cv2.imshow("New opening", opening_color)
#                 cv2.waitKey(10)
#                     
#                 
#                 print "Mat:"
#                 print pt
#                 
#                 if pt:                        
#                     dists = scipy_dist.squareform(scipy_dist.pdist(pt))
#                     print "Dist Mat:"                
#                     print dists
#==============================================================================
                    
                balls_max_dist = 100
                
                    
#==============================================================================
#         
#                     #self.A.append(pt) #/camera_link
#                     #TODO 
#                     # Read link position from tf
#                     #self.B.append([1,1,1]) 
#                 print "-----TEST----"
#                 print pt[0]
#                 print pt[0][0]
#         
#                 if len(self.A)>4:
#                     self.calibrate3d()
#                     return
#                     
#                 print ""
#                 time.sleep(0.5)                              
#==============================================================================
                

def main(argv):
    rospy.init_node("simple_kinect_extrinsics_calibration",anonymous=True)
    if rospy.has_param("/use_sim_time"):
        rospy.logwarn("Using simulation time")
        while not rospy.Time.now():
            pass # tsim syncing
       
    kinect_name = rospy.get_param('~kinect_name')
    camera_frame = rospy.get_param('~camera_frame')
    output_file = rospy.get_param('~output_file')
    
    calib = KinectSinglePointsCalibrationExtrinsics(kinect_name, camera_frame, output_file)
    calib.start()
    rospy.spin()
    calib.save_calibration()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)
