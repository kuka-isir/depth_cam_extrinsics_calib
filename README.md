Depth camera extrinsics calibration
=====
[![Build Status](https://travis-ci.org/kuka-isir/depth_cam_extrinsics_calib.svg?branch=master)](https://travis-ci.org/kuka-isir/depth_cam_extrinsics_calib)

This is a simple tool to estimate the extrinsics (world to kinect) of a depth sensor.
### Calibration instructions

#### Kinect driver

```
roslaunch openni_launch openni.launch camera:=camera
```

#### Intrinsics calibration

```bash
rosrun camera_calibration cameracalibrator.py image:=/camera/rgb/image_raw camera:=/camera/rgb --size 6x8 --square 0.067
```

#### Extrinsics calibration

Launch openni (1 or 2) **with** depth registration.

```bash
roslaunch openni_launch openni.launch depth_registration:=true camera:=kinect
```
Then launch the calibration script (example) : 
```bash
rosrun kinect_extrinsics_simple simple_chessboard_calib.py kinect /base_link 6 8 0.067 0 0 0
# Arguments are : camera_name base_name chess_width chess_height square_size x0 y0 z0
```
>Note : width is the number of squares in the desired 'x' direction, height in the 'y'.
>Also, it is preferable to set x0,y0,z0 to zero and publish a static transform from the real base to the upper left corner of the chessboard

