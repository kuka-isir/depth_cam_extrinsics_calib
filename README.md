Depth camera extrinsics calibration
=====
[![Build Status](https://travis-ci.org/kuka-isir/depth_cam_extrinsics_calib.svg?branch=master)](https://travis-ci.org/kuka-isir/depth_cam_extrinsics_calib)

This package provides tools for calibrating the camera pose and the camera depth offsets by using ROS and OpenCV

## Depth offsets calibration (for openni and openni2)
If you need a precise depth information, you have realized that by default the depth given by openni(2) is not perfectly accurate and allows you to specify in its driver `z_offset_mm` and `z_offset_scaling` so that: 

`z = z * z_offset_scaling + z_offset_mm`

Currently openni(2) doesn't give any way of calibrating these two values. Lucky you this package does.
What happens here is that I use a simple ARMarker and the ROS package ar-track-alvar, I get a set of several poses of the tag and compare the real depth value given by the ARMarker and the depth given by the RGBD sensor (Kinect, Xtion, ...).

#### Instructions
1 - Calibrate the instrinsics :

http://wiki.ros.org/openni_launch/Tutorials/IntrinsicCalibration

2 - Get the `depth_cam_tools` package which contains classes for running this code :

https://github.com/kuka-isir/depth_cam_tools

3 - Get `ar-track-alvar` : 

`sudo apt-get install ros-$ROS_DISTRO-ar-track-alvar`

4 - Create a ARMarker, print it, stick it to a hard object

5 - Run the RGBD sensor **with** depth registration: 

`roslaunch openni_launch openni.launch depth_registration:=true`

6 - Run the depth offsets calibration : 

`roslaunch depth_cam_extrinsics_calib calib_depth_offsets.launch`

7 - Move the ARMarker in the front of the sensor

8 - Kill the node with Ctrl+C, it will ask you if you want to save the calibration or not to the file specified in the args

###### Arguments
- *kinect_type* (string, default: Kinect) 

    Can be Kinect or Xtion, no difference so far

- *kinect_name* (string, default: camera) 

- *output_file* (string, default: $(find kinects_human_tracking)/config/$(arg kinect_name)_depth_offsets.yaml)

    Path to the file you where you want to save the offsets calibration

- *nb_points* (int, default: 12)

    Number of points to use for calibration

- *dz* (double, default: 0.05)
 
    Minimum distance between each points used for calibration

- *marker_id* (int, default: 0)

    ID of the tag you are using

- *marker_size* (double, default: 4.0)

    Marker size in cm

- *cam_image_topic* (string, default: $(arg kinect_name)/rgb/image_mono)

- *cam_info_topic* (string, default: $(arg kinect_name)/rgb/camera_info)


## Sensor pose calibration (checkerboard)

Launch openni (1 or 2) **with** depth registration.

```
roslaunch openni_launch openni.launch depth_registration:=true camera:=kinect
```
Then launch the calibration script (example) : 
```
rosrun kinect_extrinsics_simple simple_chessboard_calib.py kinect /base_link 6 8 0.067 0 0 0
# Arguments are : camera_name base_name chess_width chess_height square_size x0 y0 z0
```
>Note : width is the number of squares in the desired 'x' direction, height in the 'y'.
>Also, it is preferable to set x0,y0,z0 to zero and publish a static transform from the real base to the upper left corner of the chessboard

## Sensor pose calibration (reflexive markers such as Optitrack's or Vicon's)

If you happen to have three reflective markers, a 3D printer and a robot, you can place the 3 markers on the robot and use them to calibrate the camera instead. Feel free to ask for help in the issue's tab

`roslaunch depth_cam_extrinsics_calib calib_extrinsics.launch`

###### Arguments
- *kinect_type* (string, default: Kinect) 

    Can be Kinect or Xtion, no difference so far

- *kinect_name* (string, default: camera) 

- *base_frame* (string, default: /base_link) 

- *calibration_frame* (string, default: /calib_link) 
    Frame attached to the robot that is in the middle of the 3 markers

- *output_file* (string, default: $(find kinects_human_tracking)/launch/$(arg kinect_name)_extrinsics.launch)

    Path to the file you where you want to save the camera extrinsics calibration



