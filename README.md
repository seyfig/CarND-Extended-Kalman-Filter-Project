# Extended Kalman Filter
[Udacity - Self-Driving Car NanoDegree Extended Kalman Filter Project]
(https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)


## Overview
---

In this project, to aim is to implement the extended Kalman Filter in c++. LIDAR and RADAR measurements detecting a bicycle that travels around the vehicle, were provided by Udacity.

## The Project

The changes performed to implement Extended Kalman Filter can be listed as follows.

* Complete CalculateRMSE and CalculateJacobian functions in the Tools class.
* Complete Predict, Update, and UpdateEKF functions in KalmanFilter class.
* Complete FusionEKF, and ProcessMeasurement functions in FusionEKF class.


### RMSE

The RMSE values for the two files are given in the table below.

| RMSE | file 1     | file 2    |
|:----:|:----------:|:---------:|
|  px  |  0.0651648 | 0.185495  |
|  py  |  0.0605379 | 0.190302  |
|  vx  |  0.533212  | 0.487137  |
|  vy  |  0.544193  | 0.810657  |

I have also experimented to change the initialization with LASER measurements when both px and py were zero. However, this resulted in increase for the some of the RMSE values. The table for the new RMSE values, for the small number to change with zero are given in the table below. Only the second file was used.

| RMSE |0        | 0.1      | 0.01     | 0.001    | 0.0001   |
|:----:|:-------:|:--------:|:--------:|:--------:|:--------:|
|  px  |0.185495 | 0.185917 | 0.185694 | 0.185692 | 0.185692 |
|  py  |0.190302 | 0.190402 | 0.190209 | 0.190207 | 0.190207 |
|  vx  |0.487137 | 0.479301 | 0.47926  | 0.47926  | 0.47926  |
|  vy  |0.810657 | 0.828168 | 0.82795  | 0.827929 | 0.827927 |


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)