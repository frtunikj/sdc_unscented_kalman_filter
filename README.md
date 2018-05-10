# Unscented Kalman Filter

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

[//]: # (Image References)
[image0]: ./docs/UKF.png
[image1]: ./docs/EKF.png

In this Self-Driving Car Engineer Nanodegree Program project I utilize the kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. The two different types (modalities) of sensors provide/measure the followig properties:

* A lidar sensor measures the position in cartesian-coordinates (x, y).
* A radar sensor measures the position and relative velocity (the velocity within line of sight) in polar coordinates (rho, phi, drho).

The Unscented Kalman Filter (UKF) predicts the position and how fast we are going in what direction at any point in time. The UKF in this repository assumes a constant turn/yaw rate and velocity magnitude model (CTRV) for this particular system. Compared to the Extended Kalman Filter with a constant velocity model, the RMSE of the UKF should be lower especially for velocity. The CTRV model is more precise than a constant velocity model. Moreover, the UKF is also known for handling non-linear equations better than EKF. 

Detailed description of the UKF can be find in this [paper](https://www.seas.harvard.edu/courses/cs281/papers/unscented.pdf). 

The result of the prediction as well as the RMSE is depected on the figure below.

![alt text][image0]

The RMSE of the UKF is lower than the one of the [EKF](https://github.com/frtunikj/sdc_extended_kalman_filter) - see figure below.

![alt text][image1]

The Udacity code basis for the project can be find [here](https://github.com/udacity/CarND-Unscented-Kalman-Filter-Project).

## Unscented Kalman Filter Project Starter Code

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, src/ukf.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.



