# drone-navigation-super-pixels
## Description
This ROS package contains all the process of getting stereo-vision data. This is processed by a super-pixel algorithm to create plane-based maps.


The super-pixels' algorithm was developed by C. Yuheng, V. Prisacariu, and I. Reid. [1]

### New Version Changes

This new version incorporates a node to only visualize planes on a computer.

##  Requirements

* Ubuntu 18

* ROS MELODIC

* CUDA 10.0 (Important to be 10.0 not 10.1 or greater).

* ZED SDK v2.8.3. and the ROS Wrapper Pluggin

https://www.stereolabs.com/developers/release/#sdkdownloads_anchor

## Instructions

TODO

```console
$ cd ~/catkin_ws/src
$ git clone https://github.com/jpablo27/drone-navigation-super-pixels.git
$ cd ..
$ catkin_make #Wait until it compiles
```
Note: You might have to install other dependecies such as pcl-msgs, ros-pcl etc. To install them, just run:
```console
$ sudo apt-get install ros-melodic-<missing package name>
```
## REFERENCES

[1] C. Yuheng, V. Prisacariu, and I. Reid.  "gSLICr: SLIC superpixels at over 250Hz", ArXiv e-prints. 1509.04232 2015 sep
https://github.com/carlren/gSLICr
