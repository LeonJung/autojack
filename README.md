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

[Remote PC & TurtleBot SBC] Open new terminal, then enter

``` bash
$ sudo apt-get install ros-kinetic-cv-bridge ros-kinetic-vision-opencv python-opencv libopencv-dev ros-kinetic-image-proc
```

### 1.3 Calibration

#### 1.3.1 Camera Imaging Calibration

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt_image_view
```

then, click `/camera/image/compressed` or `/camera/image/` topic in the select box. If everything works fine, the screen should show you the view from the robot.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, click `camera`, adjust the parameter value that makes the camera show clean, enough bright image to you. After that, overwrite each values on to the `turtlebot3_auto_camera/calibration/camera_calibration/camera.yaml`. This will make the camera set its parameters as you set here from next launching.

#### 1.3.2 Intrinsic Camera Calibration

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [Remote PC] Print checkerboard for camera calibration on A4 size paper. The checkerboard is in `turtlebot3_auto_camera/data/checkerboard_for_calibration.pdf`. See [Calibration manual](http://wiki.ros.org/camera_calibration) and modify the parameter value written in `turtlebot3_auto_camera/launch/turtlebot3_auto_intrinsic_camera_calibration.launch`.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=calibration
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_intrinsic_camera_calibration.launch
```

- 5. [Remote PC] After finishing the calibration, intrinsic camera calibration file will be saved in `turtlebot3_auto_camera/calibration/intrinsic_calibration/camerav2_320x240_30fps.yaml`. 

#### 1.3.3 Extrinsic Camera Calibration

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [TurtleBot3 SBC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=action
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_intrinsic_camera_calibration.launch
```

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_EX_CALIB=calibration
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_extrinsic_camera_calibration.launch
```

- 5. [Remote PC] Open new terminal, then enter

```
$ rqt
```

clicking <kbd>plugins</kbd> -> <kbd>visualization</kbd> -> <kbd>Image view</kbd> on the top of the screen will make extra monnitor for camera view. Make 2 extra monitor in the rqt plate by following it. Then, choose `/camera/image_extrinsic_calib/compressed` and `/camera/image_projected_compensated` topics on each of extra monitors. If everything works fine, one of the screen will show the image with red rectangle, and other one will show the ground projected view (bird's eye view) which is based on the one another.

- 6. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `/camera/image_projection` and `/camera/image_compensation_projection` that carries out visual modifications on the image. The parameter `image_projection` will change the shape of the red rectangle of `/camera/image_extrinsic_calib/compressed` image. Intrinsic camera calibration will transform the image surrounded by the red rectangle, and will show the image that looks from over the lane. After that, overwrite each values on to the `yaml` files in  `turtlebot3_auto_camera/calibration/extrinsic_calibration/`. This will make the camera set its parameters as you set here from next launching.

#### 1.3.4 Settings for Recognition

Until now, all the preprocess of image must have been tested. 

- 1. [Remote PC] Open new terminal, then enter

``` bash
$ roscore
```

- 2. [TurtleBot SBC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_camera_pi.launch
```

- 3. [TurtleBot3 SBC] Open new terminal, then enter

``` bash
$ export AUTO_IN_CALIB=action
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_intrinsic_camera_calibration.launch
```

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_EX_CALIB=action
$ roslaunch turtlebot3_auto_camera turtlebot3_auto_extrinsic_camera_calibration.launch
```

From now, the following descriptions will mainly adjust `feature detector / color filter` for object recognition. Every adjustment after here is independent to each other's process. However, to make sure, if you want to adjust each parameters in series, finish every adjustment perfectly, then continue to next.

##### 1.3.4.1 Lane Detection

- 1. Put the robot on the lane. If you placed the robot correctly, `yellow line` should be placed on the left side of the robot, and of course, `white line` should be placed on the right side of the robot. Make sure that `turtlebot3_robot` node of `turtlebot3_bringup` package is not yet launched. If it is on running, the robot will suddenly runs on the track. 


- 2. [Remote PC] Open new terminal, then enter

``` bash
$ export AUTO_DT_CALIB=calibration
$ roslaunch turtlebot3_auto_detect turtlebot3_auto_detect_lane.launch
```

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt
```

clicking <kbd>plugins</kbd> -> <kbd>visualization</kbd> -> <kbd>Image view</kbd> on the top of the screen will make extra monnitor for camera view. Make 3 extra monitor in the rqt plate by following it. Then, choose `/detect/image_yellow_lane_marker/compressed`, `/detect/image_lane/compressed` and `/detect/image_white_lane_marker/compressed` topics on each of extra monitors. If everything works fine, left and right screen will show the filtered image of the yellow line and the white line, and the center screen will show the lane of where the robot should go. In the calibration mode, left and right screen will show white, and the center screen may show abnormal result. From here, you should adjust the filter parameters to show up correct lines and the direction.

- 4. [Remote PC] Open new terminal, then enter

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

then, adjust the parameter value in `/camera/image_projection` and `/camera/image_compensation_projection` that carries out visual modifications on the image. The parameter `image_projection` will change the shape of the red rectangle of `/camera/image_extrinsic_calib/compressed` image. Intrinsic camera calibration will transform the image surrounded by the red rectangle, and will show the image that looks from over the lane. After that, overwrite value on to the `lane.yaml` file in  `turtlebot3_auto_detect/param/lane/`. This will make the camera set its parameters as you set here from next launching.

Tip: calibration process of line color filtering is sometimes so-so difficult because of your physical environment which includes the luminance of light in the room, etc. Hence, you should have patience to carry out this procedure. To make everything quickly, put the value of `turtlebot3_auto_detect/param/lane/lane.yaml` on the reconfiguration parameter, then start calibration. Calibrate hue low - high value at first. (1) Hue value means the color, and every colors, like `yellow`, `white`, have their own region of hue value (refer to hsv map). Then calibrate saturation low - high value. (2) Every colors have also their own field of saturation. Finally, calibrate the lightness low - high value. (3) In the source code, however, have auto-adjustment function, so calibrating lightness low value is meaningless. Just put the lightness high value to 255. Clearly filtered line image will give you clear result of the lane. 

- 5. [Remote PC] After overwriting the calibration file, close `rqt_rconfigure` and `turtlebot3_auto_detect_lane`, then enter

``` bash
$ export AUTO_DT_CALIB=action
$ roslaunch turtlebot3_auto_detect turtlebot3_auto_detect_lane.launch
```

- 6. Check if the results come out well by entering 

[Remote PC] 
``` bash
$ roslaunch turtlebot3_auto_control turtlebot3_auto_control_lane.launch
```

[TurtleBot3 SBC] 
``` bash
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

After entering these commands, the robot will kick-off to run. 

##### 1.3.4.2 Traffic Sign

- 1. Traffic sign detection needs some pictures of the traffic sign. Take their pictures by using `rqt_image_view` node and edit their size, shape by any of `photo editor` available in linux. The node finds the traffic sign with `SIFT algorithm`, so if you want to use your customized traffic signs ( which is not introduced in the `autorace_track`), just be aware of `More edges in the traffic sign gives better recognition results from SIFT`. 

- 2. Put the robot on the lane. At this time, the traffic sign should be placed to where the robot can see it easily. Make sure that `turtlebot3_robot` node of `turtlebot3_bringup` package is not yet launched. If it is on run, the robot may suddenly run on the track.

- 3. [Remote PC] Open new terminal, then enter

```
$ rqt_image_view
```

then, click `/camera/image_compensated` topic in the select box. If everything works fine, the screen should show you the view from the robot.

- 4. [Remote PC] Take the picture by <kbd>alt</kbd> + <kbd>print screen</kbd>, edit the captured with your preferred photo editor. After that, place the picture to `[where the autorace_machine you've placed]/turtlebot3_auto/turtlebot3_auto_detect/file/detect_sign/` and rename it as you want. (Although, you should change the file name written in the source `detect_sign.py`, if you want to change the default file names.)


- 5. [Remote PC] Open new terminal, then enter

``` bash
$ roslaunch turtlebot3_auto_detect turtlebot3_auto_detect_sign.launch
```


- 6. [Remote PC] Open new terminal, then enter

```
$ rqt_image_view
```

then, click `/detect/image_traffic_sign/compressed` topic in the select box. If everything works fine, the screen will show the result of traffic sign detection, if it succeeds to recognize it.

##### 1.3.4.3 Traffic Light

##### 1.3.4.4 Parking Lot

##### 1.3.4.5 Level Crossing

##### 1.3.4.6 Tunnel
