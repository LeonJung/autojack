![](https://github.com/ROBOTIS-GIT/ROBOTIS-Documents/blob/master/wiki-images/AutoRace/autorace_pics/autorace_rbiz_challenge_2017_robots_1.png)

# TurtleBot3 Auto on AutoRace 2017 - Source codes
Sources for TurtleBot3 Auto - AutoRace 2017

![Picture of Pi Camera mounted TurtleBot3 Burger]

This source code is for AutoRace 2017. 

![](https://youtu.be/sp02Q4FHOWo)
## 1. Preparations

### 1.1 Requirements

* TurtleBot3 Burger
  * ROS and dependent ROS packages needs to be installed in the robot
  * All functions of TurtleBot3 Burger which is described in [TurtleBot3 E-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) needs to be tested before running TurtleBot3 Auto source code

* Remote PC (Laptop, Desktop, etc.)
  * ROS and dependent ROS packages needs to be installed in the computer
  * All functions of TurtleBot3 Burger which is described in [TurtleBot3 E-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) needs to be tested before running TurtleBot3 Auto source code

* Add-ons on TurtleBot3 Burger
  * Raspberry Pi Camera Type G (Fisheye Lens) : Available [Here](https://www.waveshare.com/rpi-camera-g.htm)
    * See `Features of 4 screw holes` in the page very carefully before mounting on the frame of any conductive materials
  * Raspberry Pi Camera Mount

![Picture of mounted Pi Camera]

* Track structure and Accessories, such as Traffic Signs, Traffic Lights, and other objects.
  * Get Sources of AutoRace Referee system from [autorace_referee](https://github.com/ROBOTIS-GIT/autorace_referee)
  * Get 3D CAD model data of the race track from [autorace_track](https://github.com/ROBOTIS-GIT/autorace_track)

### 1.2 Install Additional Dependent Packages

[Remote PC & TurtleBot SBC]

``` bash
$ sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-vision-opencv python-opencv libopencv-dev ros-kinetic-image-proc
```

### 1.3 Calibration

#### 1.3.1 Camera Imaging Calibration

- 1. [Remote PC]

``` bash
$ roscore
```

- 2. [TurtleBot SBC]

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [Remote PC]

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, click `camera`, adjust the parameter value that makes the camera show clean, enough bright image to you. After that, overwrite each values on to the `turtlebot3_auto_camera/calibration/camera_calibration/camera.yaml`. This will make the camera set its parameters as you set here from next launching.

#### 1.3.2 Intrinsic Camera Calibration

- 1. [Remote PC]

``` bash
$ roscore
```

- 2. [TurtleBot SBC]

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [Remote PC] Print checkerboard for camera calibration on A4 size paper. The checkerboard is in `turtlebot3_auto_camera/data/checkerboard_for_calibration.pdf`. See [Calibration manual](http://wiki.ros.org/camera_calibration) and modify the parameter value written in `turtlebot3_auto_camera/launch/turtlebot3_auto_intrinsic_camera_calibration.launch`.

``` bash
$ export TURTLEBOT3_AUTO_IN_CALIB=calibration
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_intrinsic_camera_calibration.launch
```

- 4. [Remote PC] After finishing the calibration, intrinsic camera calibration file will be saved in `turtlebot3_auto_camera/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml`. 

#### 1.3.3 Extrinsic Camera Calibration

- 1. [Remote PC]

``` bash
$ roscore
```

- 2. [TurtleBot SBC]

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [TurtleBot3 SBC] 

``` bash
$ export TURTLEBOT3_AUTO_IN_CALIB=action
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_intrinsic_camera_calibration.launch
```

- 4. [Remote PC]

``` bash
$ export TURTLEBOT3_AUTO_EX_CALIB=calibration
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_extrinsic_camera_calibration.launch
```

- 5. [Remote PC]

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `camera/image_projection`, `camera/image_compensation` and `camera/image_compensation_projection` that carries out visual modifications on the image. After that, overwrite each values on to the `yaml` files in  `turtlebot3_auto_camera/calibration/extrinsic_calibration/`. This will make the camera set its parameters as you set here from next launching.

#### 1.3.4 Settings for Recognition

Until now, all the preprocess of image must have been tested. 

- 1. [Remote PC]

``` bash
$ roscore
```

- 2. [TurtleBot SBC]

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [TurtleBot3 SBC] 

``` bash
$ export TURTLEBOT3_AUTO_IN_CALIB=action
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_intrinsic_camera_calibration.launch
```

- 4. [Remote PC]

``` bash
$ export TURTLEBOT3_AUTO_EX_CALIB=action
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_extrinsic_camera_calibration.launch
```

From now, the following descriptions will mainly adjust `feature detector / color filter` for object recognition. 

##### 1.3.4.1 Traffic Light

##### 1.3.4.2 Parking Lot

##### 1.3.4.3 Level Crossing

##### 1.3.4.4 Tunnel
