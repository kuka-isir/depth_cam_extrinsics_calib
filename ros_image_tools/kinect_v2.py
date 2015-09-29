#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Mon June 29 11:16:38 2015

@author: Jimmy Da Silva <jimmy.dasilva@isir.upmc.fr>
"""
import time
import ros_image_subscriber as rib
import numpy as np
from geometry_msgs.msg import PointStamped
import tf
import rospy
from threading import Thread
from sensor_msgs.msg import CameraInfo
import cv2

def get_output_list(cmd,timeout=None):
    import subprocess,time
    output  = []
    start = time.time()
    while not output:
        try:
            output = subprocess.check_output(cmd, shell=True,universal_newlines=True).split('\n')
        except:
            if timeout:
                if time.time() - start > timeout:
                    break
                else:
                    time.sleep(0.1)
            else:
                break
    return list(filter(None, output))
    
class Kinect_v2:
    def __init__(self,camera_name='/camera',serial='501427243142',queue_size=1,compression=True,use_rect=True,use_ir=False):
       
        try:
            rospy.node_init("depth_sensor",anoymous=True)
        except: pass
    
        if not camera_name[0]=="/":
            camera_name = "/"+camera_name
        self.camera_name = camera_name
        # Waiting for service to be available, like the camera calibrator
        
        camera_info_service = get_output_list("rosservice list | grep "+camera_name+"_points_xyzrgb_sd ",timeout=30.0)[0]
        rospy.loginfo(self.camera_name+" waiting for "+camera_info_service)
        rospy.wait_for_service(camera_info_service,timeout=30.0)

        self.use_ir = use_ir
        
        self.serial = serial
            
        rect=""
        if use_rect:
            rect="_rect"
        
        self.rgb_topic      = camera_name+'/sd/image_color'+rect
        
        self.depth_topic    = camera_name+'/sd/image_depth'+rect

        self.ir_topic       = camera_name+'/sd/image_ir'+rect
            
        ## Frames
        self.depth_optical_frame    = camera_name+'_ir_optical_frame'
        
        self.link_frame             = camera_name+'_link'
        
        self.rgb_optical_frame      = camera_name+'_rgb_optical_frame'

        ## Get Intrinsics
        self.depth_camera_info=self.get_camera_info(camera_name,'ir')
        self.depth_th = rib.ROSImageSubscriber(self.depth_topic,queue_size=queue_size,use_compression=compression)             
        self.depth_th.start()
        
        if not self.use_ir:
            self.rgb_camera_info=self.get_camera_info(camera_name,'color')
            self.rgb_th = rib.ROSImageSubscriber(self.rgb_topic,queue_size=queue_size,use_compression=compression)
            self.rgb_th.start()
        else:
            self.ir_camera_info=self.get_camera_info(camera_name,'ir')
            self.ir_th = rib.ROSImageSubscriber(self.ir_topic,queue_size=queue_size,use_compression=compression)
            self.ir_th.start()
                        
        self.tf = tf.TransformListener()
        #self.cloud_event = Event()
        
    def get_camera_info(self,camera_name,img_name='ir'):
        import yaml
        import os
        camera_info = CameraInfo()
        file_url = ''
        try : 
            file_url = rospy.get_param(camera_name+'_bridge/calib_path')
            print 'File_url in try : '+ file_url
        except Exception,e: 
            print e
            
        file_url += self.serial+'/calib_'+img_name+'.yaml' 
        print 'File url : '+file_url

        if not os.path.exists(file_url):
            camera_info.K = np.array([364.4180932023573, 0.0, 252.45583571288873, 0.0, 364.2169026898533, 208.49418432813215, 0.0, 0.0, 1.0])
            camera_info.D = np.array([0.11242015040414605, -0.3073512261743915, 0.0019339854599407216, -0.0014436804093048005, 0.12289934026554251])
            camera_info.P =  np.matrix([364.4180932023573, 0.0, 252.45583571288873, 0.0, 0.0, 364.2169026898533, 208.49418432813215, 0.0, 0.0, 0.0, 1.0, 0.0])
            rospy.logwarn( "No camera info found at url ["+file_url+"], using default values.\n Consider setting the *_info_url")
            return camera_info
    
        print 'Loading camera '+img_name+' info at:',file_url
        with open(file_url, 'r') as f:
            calib = yaml.load(f)
            #print 'calib '+calib
            camera_info.K = np.matrix(calib["cameraMatrix"]["data"])
            camera_info.D = np.array(calib["distortionCoefficients"]["data"])
            camera_info.R = np.matrix(calib["rotation"]["data"])
            camera_info.P = np.matrix(calib["projection"]["data"])
            camera_info.P = np.delete(camera_info.P, [15,14,13,12])
            print camera_info
        return camera_info

    def mouse_callback_spin_once(self):
        self.depth_th.mouse_callback_spin_once()        
        if not self.use_ir:
            self.rgb_th.mouse_callback_spin_once()
        else:
            self.rgb_th.mouse_callback_spin_once()        

    def register_mouse_callbacks(self,function):
        self.depth_th.register_mouse_callback(function)
        if not self.use_ir:
            self.rgb_th.register_mouse_callback(function)
        else:
            self.ir_th.register_mouse_callback(function)              

    def get_rgb_window_name(self):
        return self.rgb_th.get_window_name()

    def get_ir_window_name(self):
        return self.ir_th.get_window_name()

    def get_depth_window_name(self):

        return self.depth_th.get_window_name()
            
    def release(self):
        self.release_depth()
        if not self.use_ir:
            self.release_rgb()
        else:
            self.release_ir()
        

    def locked(self):
        if not self.use_ir:            
            return self.rgb_th.locked() or self.depth_th.locked()
        else:
            return self.ir_th.locked() or self.depth_th.locked()

    def lock(self):
        self.lock_depth()        
        if not self.use_ir:
            self.lock_rgb()        
        else:        
            self.lock_ir()

    def lock_rgb(self):
        self.rgb_th.lock()
        
    def lock_ir(self):
        self.ir_th.lock()

    def lock_depth(self):
        self.depth_th.lock()

    def release_rgb(self):
        self.rgb_th.release()
        
    def release_ir(self):
        self.ir_th.release()

    def release_depth(self):
        self.depth_th.release()

    def transform_point(self,numpoint,target_frame,source_frame):
        p = PointStamped()
        p.header.frame_id = source_frame
        p.point.x = numpoint[0]
        p.point.y = numpoint[1]
        p.point.z = numpoint[2]
        p_out = [np.nan]*3
        try:
            self.tf.waitForTransform(target_frame,source_frame,rospy.Time(0),rospy.Duration(5.0))
            geo_out = self.tf.transformPoint(target_frame, p).point
            p_out = np.array([ geo_out.x,geo_out.y,geo_out.z])
        except tf.Exception,e:
            print e

        return p_out
        
    def get_closest_pt2d(self,pt3d,list_pt2d=None):
        if not isinstance(pt3d,list):
            pt3d = np.asarray(pt3d)

        depth = self.get_depth()
        pmin_all=np.empty((len(pt3d),2))

        dmin_all=[np.inf]*len(pt3d)
        #cloud = np.empty((depth.shape[0],depth.shape[1],3)
        #n=depth.shape[0]*depth.shape[1]
        for i in xrange(depth.shape[0]):
            for j in xrange(depth.shape[1]):
                if self.cloud_event.is_set():
                    print "Cancelling current process"
                    return []
                p = [self.depth_to_world(i,j,depth,transform_to_camera_link=True)]*len(pt3d)
                dall = np.apply_along_axis(np.linalg.norm, 1, pt3d-p)
                for k in xrange(len(dall)):
                    if dall[k]<dmin_all[k]:
                        dmin_all[k] = dall[k]
                        pmin_all[k] = [i,j]
            #print i*depth.shape[1]*100/n,"%"
        if not list_pt2d:
            return pmin_all
        else:
            list_pt2d = pmin_all

    def world_to_depth(self,pt):
        if not self.use_ir:
            projMatrix = np.matrix(self.rgb_camera_info.P).reshape(3,4)
            distCoeffs = np.matrix(self.rgb_camera_info.D)
        else:
            projMatrix = np.matrix(self.ir_camera_info.P).reshape(3,4)
            distCoeffs = np.matrix(self.ir_camera_info.D)
            
        cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles = cv2.decomposeProjectionMatrix(projMatrix)

        
        rvec,_ = cv2.Rodrigues(rotMatrix)

        imgpoints2, _ = cv2.projectPoints(np.array([pt]), rvec, np.zeros(3),cameraMatrix, distCoeffs)

        result = imgpoints2[0][0]
        return result

    def depth_to_world(self,x,y,depth_img=None,transform_to_camera_link=True):
        if not self.use_ir:
            projMatrix = np.matrix(self.rgb_camera_info.P).reshape(3,4)
        else:
            projMatrix = np.matrix(self.ir_camera_info.P).reshape(3,4)
            
        cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles = cv2.decomposeProjectionMatrix(projMatrix)
        #cameraMatrix = self.depth_camera_info.K
        fx_d = cameraMatrix[0,0]
        fy_d = cameraMatrix[1,1]
        cx_d = cameraMatrix[0,2]
        cy_d = cameraMatrix[1,2]

        if depth_img is None:
            depth_img = self.get_depth()
        result = [np.nan]*3
        
        try:
            if depth_img.size:
                z = (depth_img[y][x])[0]
                if (z == 0):
                    return [np.nan]*3
                result = [(x - cx_d) * z / fx_d ,(y - cy_d) * z / fy_d, z ]
        except Exception,e: 
            print e

        if transform_to_camera_link:
            return self.transform_point(result,self.link_frame,self.depth_optical_frame)
        else:
            return result

    def is_ready(self):
        if self.use_ir:
            return self.ir_th.has_received_first[0]
        else:
            return self.rgb_th.has_received_first[0]
            
    def __wait_until_ready(self):
        while not self.is_ready() and self.is_alive():
            if not self.use_ir:
                if not self.rgb_th.has_received_first[0]:
                    rospy.loginfo(self.camera_name+' waiting for '+self.rgb_topic+' to be ready')
            else:
                if not self.ir_th.has_received_first[0]:
                    rospy.loginfo(self.camera_name+' waiting for '+self.ir_topic+' to be ready')
            if not self.depth_th.has_received_first[0]:
                rospy.loginfo(self.camera_name+' waiting for '+self.depth_topic+' to be ready')
            time.sleep(1.0)
        rospy.loginfo(self.camera_name+' ready !')
        
    def wait_until_ready(self,timeout=5.0):
        th = Thread(target=self.__wait_until_ready)
        th.start()
        th.join(timeout=timeout)

    def get_rgb(self,blocking=True):
        return self.rgb_th.get_image(blocking=blocking)
        
    def get_ir(self,blocking=True):
        return self.ir_th.get_image(blocking=blocking)

    def get_depth(self,blocking=True):
        return self.depth_th.get_image(blocking=blocking)

    def show_rgb(self):
        self.rgb_th.show()
        
    def show_ir(self):
        self.ir_th.show()

    def show_depth(self):
        self.depth_th.show()

    def is_alive(self):
        if self.use_ir:
            return self.ir_th.is_alive()
        else:
            return self.rgb_th.is_alive()
            
    def stop(self):
        #self.depth_registered_th.stop()
        self.depth_th.stop()
        self.rgb_th.stop()
        self.ir_th.stop()
