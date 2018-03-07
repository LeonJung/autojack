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

#### 1.2.1 Remote PC Setup

``` bash
$ sudo apt-get install 
```

#### 1.2.2 TurtleBot3 Burger SBC Setup

``` bash
$ sudo apt-get install
```

### 1.3 Calibration

#### 1.3.1 Camera Imaging Calibration

#### 1.3.2 Intrinsic Camera Calibration

#### 1.3.3 Extrinsic Camera Calibration

#### 1.3.4 Settings for Recognition

