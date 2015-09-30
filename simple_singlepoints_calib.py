#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 03 15:47:00 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""

from ros_image_tools.kinect_v2 import Kinect_v2
from ros_image_tools.kinect import Kinect
from ros_image_tools.tf_broadcaster import TfBroadcasterThread
import rospy
import time
import cv2
from cv2 import estimateAffine3D
import tf
import tf_conversions
import math
import argparse,textwrap,sys
from geometry_msgs.msg import PointStamped
import numpy as np
from tf.transformations import quaternion_from_matrix
from threading import Thread
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
    def __init__(self,kinect_type, kinect_name,base_frame,calibration_frame,serial,output_file=None):
        Thread.__init__(self)
        if kinect_name[-1] == '/':
            kinect_name = kinect_name[:-1]
        self.output_file_path = output_file
        
        if (kinect_type == "Kinect2") or (kinect_type == "Kinectv2") or (kinect_type == "Kinect_v2"):
            self.kinect_type = "Kinect2"
            print "Loading Kinect2 with serial : "+serial 
            self.kinect = Kinect_v2(kinect_name,serial,queue_size=10,compression=False,use_rect=True,use_ir=True)
        elif (kinect_type == "Kinect") or (kinect_type == "Kinect1") or (kinect_type == "Kinectv1") or (kinect_type == "Kinect_v1"):
            self.kinect_type = "Kinect"
            print "Loading Kinect1 with serial : "+serial
            self.kinect = Kinect(kinect_name,queue_size=10,compression=False,use_rect=True,use_depth_registered=False,use_ir=True)
        else:
            print "ERROR: Kinect type must be Kinect2 or Kinect"
            return       
        
        self.kinect_name = kinect_name
        self.base_frame = base_frame
        self.calibration_frame = calibration_frame
        self.transform_name = 'calib_'+self.kinect_name
        self.kinect.wait_until_ready(1.0)
        
        self.depth_pt_pub = rospy.Publisher(self.kinect_name+'/calibration/pts_depth',PointCloud,queue_size=10)
        self.world_pt_pub = rospy.Publisher(self.kinect_name+'/calibration/pts_calib',PointCloud,queue_size=10)
        self.calib_pt_pub = rospy.Publisher(self.kinect_name+'/calib_pt',PointStamped,queue_size=10)
        self.robot_pt_pub = rospy.Publisher(self.kinect_name+'/robot_pt',PointStamped,queue_size=10)

        self.A=[]
        self.B=[]         
        self.pt2d=[]
        self.pt2d_fit=[]
        self.single_pt_pos=[]
        
        self.saved_pts = []
        
        self.tf_thread = TfBroadcasterThread(self.kinect.link_frame,self.base_frame)
        
            
    def calibrate3d(self):
        # print self.A
        A = np.matrix(self.A)
        B = np.matrix(self.B)
        
#==============================================================================
# 
#         ########### Test RANSAC OpenCV affine estimation ###########
#         print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!'
#         (_,transMatOut,_) = cv2.estimateAffine3D(A, B, ransacThreshold=3.0, confidence=0.99)        
#         transMatOut = transMatOut.tolist()
#         transMatOut.append([0,0,0,1])
#         transMatOut = np.array(transMatOut)
#         quaternion  = tf_conversions.transformations.quaternion_from_matrix(transMatOut)
#         translation = tf_conversions.transformations.translation_from_matrix(transMatOut)
#         print transMatOut
#         print "Translation - Rotation"
#         print translation, quaternion
#         print '!!!!!!!!!!!!!!!!!!!!!!!!!!!!'        
#         
#         # Send the transform to ROS
#         self.tf_thread.set_transformation(translation,quaternion)
#==============================================================================
        

        ret_R, ret_t = rigid_transform_3D(A, B)
        new_col = ret_t.reshape(3, 1)
        tmp = np.append(ret_R, new_col, axis=1)
        aug=np.array([[0.0,0.0,0.0,1.0]])
        translation = np.squeeze(np.asarray(ret_t))
        T = np.append(tmp,aug,axis=0)
        quaternion = quaternion_from_matrix(T)

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
        
        self.static_transform = '<node pkg="tf" type="static_transform_publisher" name="'+self.transform_name+'" args="'\
        +' '.join(map(str, translation))+' '+' '.join(map(str, quaternion))+' '+self.base_frame+' '+self.kinect.link_frame+' 100" />'

        self.depth_pt_pub.publish(self.get_prepared_pointcloud(A,self.kinect.link_frame))
        self.world_pt_pub.publish(self.get_prepared_pointcloud(B,self.base_frame))

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
        if not self.static_transform or not self.output_file_path:
            print 'Not saving files'
            return
        if query_yes_no("Do you want to save "+str(self.output_file_path)):
            print "Saving file ",self.output_file_path
            try:
                with open(self.output_file_path,'r') as f:
                    with open(self.output_file_path+'.bak','w') as fbak:
                        print self.output_file_path,' already exists, creating backup file.'
                        fbak.write(f.read())
            except: pass
            with open(self.output_file_path,'w') as f:
                print self.static_transform
                f.write("""<launch>
   """+self.static_transform+
"""
</launch>
""")
            print "File saved."
        else:
            print "Not saving calibration."
        return    
                    
    def nothing(x,c):
        pass
    
    def start(self):
        self.tf_thread.start()
        img_shape = np.array(self.kinect.get_ir(blocking=False)).shape
        self.pixels_used = np.zeros((img_shape[0],img_shape[1]))
        
        cv2.namedWindow('Thresholding')
        cv2.createTrackbar('Threshold','Thresholding',160,255,self.nothing)

        listener = tf.TransformListener()
        
        while not rospy.is_shutdown():
            self.kinect.show_ir()
            ir_img = self.kinect.get_ir(blocking=False)
            ir_array = np.array(ir_img, dtype=np.float32)
            cv2.normalize(ir_array, ir_array, 0, 1, cv2.NORM_MINMAX)
            depth_img = self.kinect.get_depth(blocking=False)
            depth_array = np.array(depth_img, dtype=np.float32)
            cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)

            ir_8u = ir_array*255
            ir_8u = ir_8u.astype(np.uint8)
            ir_8u = cv2.GaussianBlur(ir_8u ,(5,5),3)
            
            depth_8u = depth_array*255
            depth_8u = depth_8u.astype(np.uint8)
            
            thresh = cv2.getTrackbarPos('Threshold','Thresholding')
            ret, ir_8u_thresh = cv2.threshold(ir_8u,thresh,255,cv2.THRESH_BINARY)
            cv2.imshow("Thresholding", ir_8u_thresh) 

            kernel = np.ones((3,3),np.uint8)
            opening = cv2.morphologyEx(ir_8u_thresh, cv2.MORPH_OPEN, kernel)
            cv2.imshow("Opening", opening)                    
             
            # If there is the right amount of contours
            contours, _ = cv2.findContours(opening,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
            nb_contours = len(contours)                         
            if nb_contours == 3:
                pt = []
                middle_x = 0
                middle_y = 0
                radiuses = []
                x_coords = []
                y_coords = []
                detection_img = cv2.cvtColor(opening ,cv2.COLOR_GRAY2RGB)
                for i in range(nb_contours):
                    (x,y), radius = cv2.minEnclosingCircle(contours[i])
                    (x,y) = (int(x),int(y))
                    radius = int(radius)
                    cv2.circle(detection_img, (x,y), radius, (255,255,255), -1)                
                    middle_x += x/nb_contours
                    middle_y += y/nb_contours
                    radiuses.append(radius)
                    x_coords.append(x)
                    y_coords.append(y)
               
                # Leave if the balls don't seem to have the same size
                if (abs(radiuses[0]-radiuses[1])>2 or abs(radiuses[0]-radiuses[2])>2 or abs(radiuses[2]-radiuses[1])>2):
                    continue
                
                # Leave if the triangle seems to big                
                dists = []
                dists.append(math.sqrt( math.pow(x_coords[0]-middle_x,2) + math.pow(y_coords[0]-middle_y,2) ))
                dists.append(math.sqrt( math.pow(x_coords[1]-middle_x,2) + math.pow(y_coords[1]-middle_y,2) ))
                dists.append(math.sqrt( math.pow(x_coords[2]-middle_x,2) + math.pow(y_coords[2]-middle_y,2) ))
                if (dists[0]>25 or dists[1]>25 or dists[2]>25):
                    continue
                
                middle_x = int(middle_x)
                middle_y = int(middle_y)
                cv2.circle(detection_img,(middle_x,middle_y),2,(0,255,0),-1)
                cv2.imshow("Detection", detection_img)
                
                cv2.circle(depth_array,(middle_x,middle_y),2,255,-1)
                cv2.imshow("Detection_raw", depth_array)
                
                pt = self.kinect.depth_to_world(middle_x,middle_y,transform_to_camera_link=True)
                if not (True in np.isnan(pt)) and (pt is not None):
                    
                    calib_pt = PointStamped()
                    calib_pt.header.frame_id=self.kinect.link_frame
                    calib_pt.header.stamp = rospy.Time.now()
                    calib_pt.point.x = pt[0]
                    calib_pt.point.y = pt[1]
                    calib_pt.point.z = pt[2]
                    self.calib_pt_pub.publish(calib_pt)
                        
                    if self.saved_pts is not None:
                        if (len(self.saved_pts)>=5):
                            self.saved_pts.pop(0)  
                            
                        try:
                            (trans,rot) = listener.lookupTransform(self.base_frame, self.calibration_frame, rospy.Time(0))
                            robot_pt = PointStamped()
                            robot_pt.header.frame_id=self.base_frame
                            robot_pt.header.stamp = rospy.Time.now()
                            robot_pt.point.x = trans[0]
                            robot_pt.point.y = trans[1]
                            robot_pt.point.z = trans[2]
                            self.robot_pt_pub.publish(robot_pt)
                        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                            continue                          
                        self.saved_pts.append(trans)
                        
                        if (len(self.saved_pts)==5):
                            dists = scipy_dist.squareform(scipy_dist.pdist(self.saved_pts))
                            max_dist = np.amax(dists)
                            
                            # Check robot is not moving
                            if max_dist < 0.001:
                                # Check pixel hasn't been used already
                                if not self.pixels_used.item( (middle_y-1, middle_x-1) ):
                                    self.pixels_used[middle_y-1, middle_x-1] = 1
                                    print 'SAVING FOR CALIBRATION !!!'
                                    self.A.append(pt)
                                    self.B.append([trans[0],trans[1],trans[2]])
                                                                            
                                    print 'pt: ', pt
                                    print 'trans: ', trans,'\n'
                                    
                                    time.sleep(0.5)
                                    self.saved_pts=[]
                
                if len(self.A)>4:
                    self.calibrate3d()
        

def main(argv):
    rospy.init_node("simple_kinect_extrinsics_calibration",anonymous=True)
    if rospy.has_param("/use_sim_time"):
        rospy.logwarn("Using simulation time")
        while not rospy.Time.now():
            pass # tsim syncing
       
    kinect_name = rospy.get_param('~camera_name')
    base_frame = rospy.get_param('~base_frame')
    calibration_frame = rospy.get_param('~calibration_frame')
    output_file = rospy.get_param('~output_file')
    serial = rospy.get_param('~serial')
    kinect_type = rospy.get_param('~kinect_type')
    
    calib = KinectSinglePointsCalibrationExtrinsics(kinect_type, kinect_name, base_frame, calibration_frame, serial, output_file)
    calib.start()
    rospy.spin()
    calib.save_calibration()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)
