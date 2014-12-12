#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 28 11:16:38 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
"""
import time
import ros_image_subscriber as rib
import numpy as np
from geometry_msgs.msg import PointStamped
import tf
import rospy
from threading import Event
from sensor_msgs.msg import CameraInfo

class Kinect:
    def __init__(self,camera_name='/camera',queue_size=1,compression=True,use_rect=False):
        self.camera_name = camera_name
        if use_rect:
            rect="_rect"
        else:
            rect=""
        rgb_topic = camera_name+'/rgb/image'+rect+'_color'
        depth_topic = camera_name+'/depth/image'+rect
        depth_registered_topic = camera_name+'/depth_registered/image'+rect
        self.depth_optical_frame = camera_name+'_depth_optical_frame'
        self.link_frame = camera_name+'_link'
        self.depth_frame = camera_name+'_depth_frame'
        self.rgb_frame = camera_name+'_rgb_frame'
        self.rgb_optical_frame = camera_name+'_rgb_optical_frame'

        self.depth_camera_info=self.get_camera_info(camera_name,'depth')
        self.rgb_camera_info=self.get_camera_info(camera_name,'rgb')

        self.get_camera_info(camera_name)

        self.rgb_th = rib.ROSImageSubscriber(rgb_topic,queue_size=queue_size,use_compression=compression)
        self.depth_th = rib.ROSImageSubscriber(depth_topic,queue_size=queue_size,use_compression=compression)
        self.depth_registered_th = rib.ROSImageSubscriber(depth_registered_topic,queue_size=queue_size,use_compression=compression)
        self.using_registered = False
        self.rgb_th.start()
        self.depth_registered_th.start()
        self.depth_th.start()
        self.tf = tf.TransformListener()
        self.cloud_event = Event()




    def get_camera_info(self,camera_name,img_name='depth'):
        import yaml
        camera_info = CameraInfo()
        file_url = rospy.get_param(camera_name+'/driver/'+img_name+'_camera_info_url').replace('file://','')
        print 'Loading camera '+img_name+' info at:',file_url
        with open(file_url, 'r') as f:
            calib = yaml.safe_load(f.read())
            camera_info.K = calib["camera_matrix"]["data"]
            camera_info.D = calib["distortion_coefficients"]["data"]
            camera_info.R = calib["rectification_matrix"]["data"]
            camera_info.P = calib["projection_matrix"]["data"]
            print camera_info
        return camera_info

    def register_mouse_callback_runtime(self):
        self.rgb_th.register_mouse_callback_runtime()
        self.depth_th.register_mouse_callback_runtime()
        self.depth_registered_th.register_mouse_callback_runtime()

    def register_mouse_callbacks(self,function):
        self.rgb_th.register_mouse_callback(function)
        self.depth_th.register_mouse_callback(function)
        self.depth_registered_th.register_mouse_callback(function)

    def get_rgb_window_name(self):
        return self.rgb_th.get_window_name()

    def get_depth_window_name(self):
        if self.using_registered:
            return self.depth_registered_th.get_window_name()
        else:
            return self.depth_th.get_window_name()
    def release(self):
        self.release_rgb()
        self.release_depth()

    def locked(self):
        return self.rgb_th.locked() or self.depth_registered_th.locked() or self.depth_th.locked()

    def lock(self):
        self.lock_rgb()
        self.lock_depth()

    def lock_rgb(self):
        self.rgb_th.lock()

    def lock_depth(self):
        if self.using_registered:
            self.depth_registered_th.lock()
        else:
            self.depth_th.lock()

    def release_rgb(self):
        self.rgb_th.release()

    def release_depth(self):
        if self.using_registered:
            self.depth_registered_th.release()
        else:
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
        if False:#self.depth_camera_info.K:


            P = np.matrix(self.depth_camera_info.P).reshape(3,4)
            pt = np.matrix([pt[0],pt[1],pt[2],1]).T
            print "P:",P
            print "pt:",pt
            result = P*pt
            print "result:",result
            return [result[0],result[1]]

        else:
            pt = np.matrix(pt).T
            if self.depth_camera_info.K:
                fx_d = self.depth_camera_info.K[0]
                fy_d = self.depth_camera_info.K[4]
                cx_d = self.depth_camera_info.K[2]
                cy_d = self.depth_camera_info.K[5]
            else:
                fx_d = 5.9421434211923247e+02;
                fy_d = 5.9104053696870778e+02;
                cx_d = 3.3930780975300314e+02;
                cy_d = 2.4273913761751615e+02;

            #transformedPos = invR*pt + invT
            invZ = 1.0 / pt[2]

            raw_x =(pt[0] * fx_d * invZ) + cx_d
            raw_y = (pt[1] * fy_d * invZ) + cy_d
            #print "xr,yr:",raw_x,raw_y
            res_x = max(0,min(int(raw_x), 639))
            res_y = max(0,min(int(raw_y), 479))
            result=[res_x,res_y]
            return result

#==============================================================================
#     def color_to_world(self,x,y):
#         fx_rgb = 5.2921508098293293e+02
#         fy_rgb = 5.2556393630057437e+02
#         cx_rgb = 3.2894272028759258e+02
#         cy_rgb = 2.6748068171871557e+02
#
#         v = self.depth_to_world(x, y)
#         v[0] = (v[0]/9.9984628826577793e-01) - 1.9985242312092553e-02
#         v[1] = (v[1]/9.9984628826577793e-01)
#         v[2] = (v[2]/9.9984628826577793e-01) - 1.9985242312092553e-02
#         result = [np.nan]*3
#         result[0] = (v[0]*fx_rgb/v[2])+cx_rgb;
#         result[1] = (v[1]*fy_rgb/v[2])+cy_rgb;
#         result[2] = v[2];
#==============================================================================
        return result;

    def raw_depth_to_meters(self, depth_value):
        # We use depth_registered so useless function
        if depth_value < 2047:
            return 1.0 / (depth_value * -0.0030711016 + 3.3309495161)
        return 0.0

    def depth_to_world(self,x,y,depth_img=None,transform_to_camera_link=True):

        if self.depth_camera_info.K:
            fx_d = 1.0 / self.depth_camera_info.K[0]
            fy_d = 1.0 / self.depth_camera_info.K[4]
            cx_d = self.depth_camera_info.K[2]
            cy_d = self.depth_camera_info.K[5]
        else:
            fx_d = 1.0 / 5.9421434211923247e+02;
            fy_d = 1.0 / 5.9104053696870778e+02;
            cx_d = 3.3930780975300314e+02;
            cy_d = 2.4273913761751615e+02;

        if depth_img is None:
            depth_img = self.get_depth()

        result = [np.nan]*3
        try:
            if depth_img.size:
                z = depth_img[y][x]
                result = np.array([(x - cx_d) * z * fx_d ,(y - cy_d) * z * fy_d, z ])
        except: return result

        if transform_to_camera_link:
            return self.transform_point(result,self.link_frame,self.depth_optical_frame)
        else:
            return result

    def is_ready(self):
        return self.rgb_th.has_received_first[0] and (self.depth_registered_th.has_received_first[0] or self.depth_th.has_received_first[0])

    def wait_until_ready(self):
        print ""
        while not self.is_ready() and self.is_alive():
            if not self.rgb_th.has_received_first[0]:
                print 'Waiting for rgb to be ready'
            if not self.depth_th.has_received_first[0]:
                print 'Waiting for depth to be ready'
            if not self.depth_registered_th.has_received_first[0]:
                print 'Waiting for depth registered to be ready'
            time.sleep(0.5)
        print 'Kinect ready...receiving images'
        if self.depth_th.has_received_first[0]:
            print '[Using depth]'
            self.using_registered = False
        if self.depth_registered_th.has_received_first[0]:
            print '[Using depth_registered]'
            self.using_registered = True

    def get_rgb(self,blocking=True):
        return self.rgb_th.get_image(blocking=blocking)

    def get_depth(self,blocking=True):
        if self.using_registered:
            return self.get_depth_registered(blocking=blocking)
        else:
            return self.depth_th.get_image(blocking=blocking)

    def get_depth_registered(self,blocking=True):
        return self.depth_registered_th.get_image(blocking=blocking)

    def show_rgb(self):
        self.rgb_th.show()

    def show_depth(self):
        if self.using_registered:
            self.show_depth_registered()
        else:
            if self.depth_th.has_received_first[0]:
                self._show_depth()

    def show_depth_registered(self):
        if self.depth_registered_th.has_received_first[0]:
            self._show_depth_registered()

    def _show_depth(self):
        self.depth_th.show()

    def _show_depth_registered(self):
        self.depth_registered_th.show()

    def is_alive(self):
        return self.rgb_th.is_alive() or self.depth_th.is_alive() or self.depth_registered_th.is_alive()

    def stop(self):
        self.depth_registered_th.stop()
        self.depth_th.stop()
        self.rgb_th.stop()
