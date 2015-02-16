#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 28 10:27:00 2014

@author: Antoine Hoarau <hoarau.robotics@gmail.com>
"""

from ros_image_tools.kinect import Kinect
from ros_image_tools.tf_broadcaster import TfBroadcasterThread
import rospy
import time
import cv2
import argparse,textwrap,sys
from geometry_msgs.msg import PointStamped
import numpy as np
from threading import Lock
from tf.transformations import quaternion_from_matrix
from threading import Thread,Event
from sensor_msgs.msg import PointCloud

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

def pair(x):
    if x%2==0: return 1
    else: return 0

class CyclicCounter:
    def __init__(self,size):
        self.s = size
        self.c = 0
        
    def inc(self,pas=1):
        self.c = self.c + pas
        if self.c >= self.s:
            self.c = 0
        return self.c
        
    def dec(self,pas=1):
        self.c = self.c - pas
        if self.c < 0:
            self.c = self.s - pas 
        return self.c
        
    def get(self):
        return self.c
    


class KinectChessboardCalibrationExtrinsics(Thread):
    def __init__(self,kinect_name,base_frame,chess_width,chess_height,square_size,upper_left_corner_position):
        Thread.__init__(self)
        if kinect_name[-1] == '/':
            kinect_name = kinect_name[:-1]
        self.kinect = Kinect(kinect_name,queue_size=10,compression=False,use_rect=True)
        self.kinect_name = kinect_name
        self.base_frame = base_frame
        self.transform_name = 'calib_'+self.kinect_name[1:]
        self.kinect.wait_until_ready()
        self.kinect.register_mouse_callbacks(self.mouse_callback)
        self.upper_left_corner_position = upper_left_corner_position
        
        self.whites_raw = self.compute_whites_pos_raw(chess_width,chess_height)
        self.chess_pos= self.compute_chess_pos_world(chess_width,chess_height,square_size,upper_left_corner_position)

        self.depth_pt_pub = rospy.Publisher(self.kinect_name+'/calibration/pts_depth',PointCloud)
        self.world_pt_pub = rospy.Publisher(self.kinect_name+'/calibration/pts_calib',PointCloud)

        
        self.chess_drawing=[]
        self.ch_w = chess_width
        self.ch_h = chess_height
        self.ch_sq = square_size
        self.n_white = int(self.ch_w*self.ch_h/2)
        self.current_pos=CyclicCounter(self.n_white)
        self.A=[]
        self.B=[]
        self.pt2d=[]
        self.pt2d_fit=[]
        self.lock_=Lock()
        self.event_ = Event()
        self.opencv_th = Event()
        self.final_draw2d_th=Thread()
        self.int_cnt=CyclicCounter(10)
        self.rvec = np.zeros((3,1),np.float32)
        self.tvec = np.zeros((3,1),np.float32)
        
        self.useExtrinsicGuess = False
        # Output tf threads
        self.tf_thread = TfBroadcasterThread(self.kinect.link_frame,self.base_frame)
        self.tfcv_thread = TfBroadcasterThread(self.base_frame+'_cv',self.kinect.rgb_optical_frame)
        
    
    def mouse_callback(self,event,x,y,flags,param):
        if self.lock_.locked() or self.event_.is_set():
            print "locked at ",rospy.Time.now()
            return
        
        if event == cv2.EVENT_RBUTTONUP:
            self.event_.set()
            self.current_pos.dec()
            self.event_.clear()
            
        if event == cv2.EVENT_LBUTTONUP:
            self.event_.set()            
            print '[x,y]=',x,y,
            pt = self.kinect.depth_to_world(x,y)
            if not (True in np.isnan(pt)):
                print ' => [',pt[0],pt[1],pt[2],']'

                self.pt2d.append([x,y])

                #self.lock_.acquire()
                self.A.append(pt) #/camera_link
                self.B.append(self.chess_pos[self.current_pos.get()]) ## /base_link
                #self.lock_.release()

                self.current_pos.inc()
                if len(self.A)>4:
                    #th = Thread(target=self.calibrate3d)
                    #th.start()
                    self.calibrate3d()
            self.event_.clear()        
        

    def calibrate3d(self):
        #self.lock_.acquire()
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


        self.pt2d_fit = []
        for p_orig2d,p in zip(self.pt2d,B_in_A): #chess dans /camera_link
            #print "p_orig2d:",p_orig2d
            pdepth = self.kinect.transform_point(p,self.kinect.depth_optical_frame,self.kinect.link_frame)
            #print "pdepth:",pdepth
            pfinal = self.kinect.world_to_depth(pdepth)
            #print "pfinal:",pfinal
            self.pt2d_fit.append(pfinal)

        self.depth_pt_pub.publish(self.get_prepared_pointcloud(A,self.kinect.link_frame))
        self.world_pt_pub.publish(self.get_prepared_pointcloud(B,self.base_frame))
        print ""
        self.static_transform = '<node pkg="tf" type="static_transform_publisher" name="'+self.transform_name+'" args="'\
        +' '.join(map(str, translation))+' '+' '.join(map(str, quaternion))+' '+self.base_frame+' '+self.kinect.link_frame+' 100" />'
        print self.static_transform
        print ""
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

    def draw_chessboard(self,img,curr=None,scale=15.0,offset=(20,20)):
        k = 0
        s = 1.0#self.ch_sq
        chess_width = self.ch_w
        chess_height= self.ch_h
        line_width = 2
        try:
            xline = (offset[0],offset[1]-line_width)
            yline = (offset[0]-line_width,offset[1])
            xline_end=(int(offset[0]+chess_height*s*scale),int(offset[1])-line_width)
            yline_end=(int(offset[0])-line_width,int(offset[1]+chess_width*s*scale))
            #print "OFFSETS : ",offset,xline,yline
            cv2.line(img,xline,xline_end,(0,255,0),line_width)
            cv2.line(img,yline,yline_end,(0,0,255),line_width)
            
        except Exception,e: print e
        
        whites = self.compute_whites_pos_raw(chess_width,chess_height)
        for i in xrange(0,len(whites)):
            p = whites[i]
            x = offset[0] + p[0]*scale*s
            y = offset[1] + p[1]*scale*s
            cv2.circle(img,(int(y),int(x)),2,(10,200,10),-1)

        for j in xrange(chess_height):
            for i in xrange(0,chess_width-j%2,2):


                x1 = int((i + j%2)*s*scale) +offset[0]
                y1 = int(j*s*scale) +offset[1]
                pt1 = (y1,x1)

                x2 = int((i + j%2 +1)*s*scale)+offset[0]
                y2 = int((j+1)*s*scale) +offset[1]

                pt2 = (y2,x2)

                cv2.rectangle(img,pt1,pt2,(0,0,0),-1)

        for i in xrange(0,len(whites)):
            if curr == k:
                p = whites[i]
                x = offset[0] + p[0]*scale*s
                y = offset[1] + p[1]*scale*s
                # invert x and y just for vizu
                cv2.circle(img,(int(y),int(x)),2,(0,0,255),-1)
            k=k+1

    def compute_chess_pos_world(self,chess_width,chess_height,square_size,upper_left_corner_position):
        whites = self.compute_whites_pos_raw(chess_width,chess_height)
        for i in xrange(len(whites)):
            p = whites[i]
            p[0] = upper_left_corner_position[0] + p[0]*square_size
            p[1] = upper_left_corner_position[1] + p[1]*square_size
            p[2] = upper_left_corner_position[2] + p[2]*square_size
        return whites
        
    def compute_whites_pos_raw(self,chess_width,chess_height):
        objp=[]        
        for j in xrange(chess_height):
            for i in xrange((j+1)%2,chess_width,2):
                x = i + 0.5
                y = j + 0.5
                z = 0
                objp.append([x,y,z])
        return objp

    def calibration_opencv(self,rgb,h,w,sq_size,use_pnp = True,use_ransac = True):
        self.opencv_th.set()
        #cameraMatrix = self.kinect.rgb_camera_info.K.reshape(3,3)
        projMatrix = np.matrix(self.kinect.rgb_camera_info.P).reshape(3,4)
        cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles = cv2.decomposeProjectionMatrix(projMatrix)
            
        distCoeffs = np.array(self.kinect.rgb_camera_info.D)
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.001)
        gray = cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)
        
        objp = np.zeros((h*w,3), np.float32)
        objp[:,:2] = np.mgrid[0:h,0:w].T.reshape(-1,2)
        objp = objp*sq_size
        objp[:,[0, 1]] = objp[:,[1, 0]]
        
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (h,w),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            cv2.cornerSubPix(gray,corners,(5,5),(-1,-1),criteria)
            imgpoints.append(corners)
            
            # Draw and display the corners
            cv2.drawChessboardCorners(rgb, (h,w), corners,ret)

            if use_pnp:
                if use_ransac:
                    if not self.useExtrinsicGuess:
                        rvec,tvec,_ = cv2.solvePnPRansac(objp, corners,cameraMatrix ,distCoeffs)
                        self.rvec = rvec
                        self.tvec = tvec
                        self.useExtrinsicGuess = True
                    else:
                        r,t,_ = cv2.solvePnPRansac(objp, corners,cameraMatrix ,distCoeffs,self.rvec, self.tvec, self.useExtrinsicGuess)
                        rvec = self.rvec
                        tvec = self.tvec

            else:
                ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],cameraMatrix,None,flags=cv2.CALIB_USE_INTRINSIC_GUESS)#flags=cv2.CALIB_FIX_ASPECT_RATIO)
                rvec = rvecs[0]
                tvec = tvecs[0]
                
            tvect = np.matrix(tvec).reshape(3,1)
            imgpoints2, _ = cv2.projectPoints(objp, rvec, tvec, cameraMatrix, distCoeffs)
            
            for p in imgpoints2:
                cv2.circle(rgb,(int(p[0][0]),int(p[0][1])),2,(0,12,235),1)
                
            cv2.imshow('findChessboardCorners - OpenCV',rgb)
            ret_R,_ = cv2.Rodrigues(rvec)
            
            #print tvec,rvec

            ret_Rt = np.matrix(ret_R)
            
            tmp = np.append(ret_Rt, np.array([0,0,0]).reshape(3,1), axis=1)
            aug=np.array([[0.0,0.0,0.0,1.0]])
            T = np.append(tmp,aug,axis=0)

            quaternion = quaternion_from_matrix(T)

            self.tfcv_thread.set_transformation(tvect,quaternion)
        self.opencv_th.clear()
        return 

    def save_calib(self):
        time.sleep(1.0)
        if query_yes_no("Would you like to save the calibration ?"):
            import yaml
            #yaml.load
            rospy.loginfo("Saving file at ")
            
    def start(self):
        self.tf_thread.start()
        self.tfcv_thread.start()
        
        print 'Starting calibration'

        while not rospy.is_shutdown():
            #self.kinect.lock()
            rgb = np.array(self.kinect.get_rgb(blocking=False))
            #self.kinect.release()
            
            if rgb.size:
                try:
                    if self.event_.is_set():
                        self.lock_.acquire()
                    
                    if not self.opencv_th.is_set():
                        find_chess_th = Thread(target=self.calibration_opencv,args=(np.array(rgb),self.ch_h-1,self.ch_w-1,self.ch_sq))
                        find_chess_th.start()
                    
                    cv2.putText(rgb,"Click on white square n"+str(self.current_pos.get()+1)+"/"+str(self.n_white), (2*20+15*self.ch_h,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250,250,255))
                    #cv2.putText(rgb,str(rospy.Time.now()), (150,30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (250,250,255))
                    self.draw_chessboard(rgb,self.current_pos.get(),scale=15,offset=(20,20))

                    for p in self.pt2d:
                        cv2.circle(rgb,(int(p[0]),int(p[1])),4,(0,12,235),1)

                    for p in self.pt2d_fit:
                        cv2.circle(rgb,(int(p[0]),int(p[1])),1,(235,12,25),1)
                    #self.kinect.show_rgb()
                    
                    cv2.imshow(self.kinect.get_rgb_window_name(), rgb)
                    #self.kinect.show_depth()
                    self.kinect.mouse_callback_spin_once()# should be after imshow to get the mouse cb on the window
        
                    #find_chess_th.join()
                    
                    cv2.waitKey(3)
                    
                    if not self.event_.is_set() and self.lock_.locked():
                        self.lock_.release()
                except Exception,e: print e
            time.sleep(1.0/30.0)
        #self.save_calib()

def main(argv):
    rospy.init_node("simple_kinect_extrinsics_calibration",anonymous=True)
    if rospy.has_param("/use_sim_time"):
        rospy.logwarn("Using simulation time")
        while not rospy.Time.now():
            pass # tsim syncing
    
    parser = argparse.ArgumentParser(formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description=textwrap.dedent(""" Simple Chessboard extrinsics calibration with kinect"""),
                                     epilog='Maintainer: Antoine Hoarau <hoarau.robotics AT gmail DOT com>')
    parser.add_argument('kinect_name', type=str,help='The name of the kinect (ex:/camera)')
    parser.add_argument('base_frame', type=str,help='Usually /base_link')
    parser.add_argument('chess_width', type=int,help='Number of squares in the x direction')
    parser.add_argument('chess_height', type=int,help='Number of squares in the y direction')
    parser.add_argument('square_size', type=float,help='Size of the squares in m')
    parser.add_argument('upper_left_corner_position', type=float,nargs=3,help='Position of the upper right corner to (0,0,0)')
    args,_ = parser.parse_known_args()
    print args
    calib = KinectChessboardCalibrationExtrinsics(args.kinect_name,
                                                  args.base_frame,
                                                  args.chess_width,
                                                  args.chess_height,
                                                  args.square_size,
                                                  args.upper_left_corner_position)
    calib.start()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
    exit(0)
