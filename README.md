# Kinect extrinsics calibration 

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

#### Extrinsics 

