# drone-navigation-super-pixels
##  Requirements
..*CUDA 10.0 (Important to be 10.0 not 10.1 or greater)
..*ZED developer toolkit  
https://www.stereolabs.com/developers/release/#sdkdownloads_anchor

This ROS package contains all the process of getting stereo-vision data. This is processed by a super-pixel algorithm to create plane-based maps.


The super-pixels' algorithm was developed by C. Yuheng, V. Prisacariu, and I. Reid. [1]


[1] C. Yuheng, V. Prisacariu, and I. Reid.  "gSLICr: SLIC superpixels at over 250Hz", ArXiv e-prints. 1509.04232 2015 sep
https://github.com/carlren/gSLICr
