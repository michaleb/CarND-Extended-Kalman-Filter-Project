## Extended Kalman Filter


[//]: # (Image References)

[image1]: ./IMG/Sensor_Fusion.png "Sensor Fusion flow diagram"
[image2]: ./IMG/RMSE-values.png "RMSE of predictions"
[image3]: ./IMG/radar.jpg "Radar measurements"


### Introduction

This project implements the extended Kalman filter in C++. It uses a Kalman filter, lidar and radar measurements to track a bicycle's position and velocity. A linear motion model is employed, where acceleration is assumed to be zero, to predict the transition of the object's state (position, velocity) In the case of non-linear radar measurements a Jacobian matrix is used to provide a linear approximation of its measured values (rho, phi and rho dot) for 2-D motion.

### Radar measurements
![alt text][image3]

Whereas radar has three measurements (rho, phi, rho dot) and its prediction and measurement functions are both non-linear, lidar measures position only and has two measurement (x, y) values and its prediction and measurement functions are both linear. These measurement values along with their timestamp will be fed into the Kalman filter algorithm. The measurement function used to transform the predicted state into the measurement space will depend on the type of sensor being processed at that time. However, the prediction function remains the same throughout as the constant velocity model is assumed. 

![alt text][image1]

The Kalman Filter algorithm will go through the following steps:

#### *first measurement* 
 - The filter receives initial measurements of the bicycle's position relative to the car. These measurements will come from a radar or lidar sensor.

#### *initialize state and covariance matrices*
 - The filter initializes the bicycle's position based on the first measurement. The car will then receive another sensor measurement after a time period Δt.

#### *predict*
 - The algorithm predicts where the bicycle will be after time Δt. It is assumed that there are no changes in the bicycle's velocity; thus the bicycle will have moved a distance of *(velocity * Δt)*. 

#### *update*
 - The filter compares the "predicted" location with the sensor's measurement. The predicted location and the measured location are combined to give an updated location. The Kalman filter places more weight on either the predicted location or the measured location depending on the uncertainty of each value. 

 - The car then receives another sensor measurement after a time period Δt and once again the algorithm does a predict and update step.

```cpp
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
```
Every time main.cpp calls fusionEKF.ProcessMeasurement(measurement_pack_list[k]), the code in FusionEKF.cpp will run. - If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

```cpp

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Convert radar from polar to cartesian coordinates 
      // and initialize state
      float ro = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float ro_dot = measurement_pack.raw_measurements_(2);

      ekf_.x_ << ro * cos(phi),
                 ro * sin(phi),
                 ro_dot * cos(phi),
                 ro_dot * sin(phi);
    }                   

    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      
      ekf_.x_ << measurement_pack.raw_measurements_[0], 
              measurement_pack.raw_measurements_[1], 
              0, 
              0;
    }

  // done initializing, no need to predict or update
  previous_timestamp_ = measurement_pack.timestamp_;
  is_initialized_ = true;
  return;
  }

```  
Predict and Update Steps in FusionEKF.cpp
Once the Kalman filter gets initialized, the next iterations of the for loop will call the ProcessMeasurement() function to do the predict and update steps.

```cpp
  /**
   * Prediction
   */
  
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // set the acceleration noise components
  float noise_ax = 9;
  float noise_ay = 9;

  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_(0,2) = dt;
  ekf_.F_(1,3) = dt;

  // 2. Set the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
         0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
         dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
         0, dt_3/2*noise_ay, 0, dt_2*noise_ay;


  ekf_.Predict();


  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } 

  else {
    
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

```  
In FusionEKF.cpp, you will see references to a variable called ekf_. The ekf_ variable is an instance of the KalmanFilter class. You will use ekf_ to store your Kalman filter variables (x, P, F, H, R, Q) and call the predict and update functions. Let's talk more about the KalmanFilter class.

KalmanFilter Class
kalman_filter.h defines the KalmanFilter class containing the x vector as well as the P, F, Q, H and R matrices. The KalmanFilter class also contains functions for the prediction step as well as the Kalman filter update step (lidar) and extended Kalman filter update step (radar).

You will need to add your code to kalman_filter.cpp to implement the prediction and update equations. You do not need to modify 'kalman_filter.h'.

Because lidar uses linear equations, the update step will use the basic Kalman filter equations. On the other hand, radar uses non-linear equations, so the update step involves linearizing the equations with the Jacobian matrix. The Update function will use the standard Kalman filter equations. The UpdateEKF will use the extended Kalman filter equations:

```cpp

void KalmanFilter::Predict() {
  
}
void KalmanFilter::Update(const VectorXd &z) {
  
}
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  
}
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  
}
```

![alt text][image2]



### Udacity;s original README
#### Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

In this project you will utilize a kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see the uWebSocketIO Starter Guide page in the classroom within the EKF Project lesson for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found in the classroom lesson for this project.

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protocol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


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
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar data, see the
[utilities repo](https://github.com/udacity/CarND-Mercedes-SF-Utilities) for
Matlab scripts that can generate additional data.

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project resources page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/382ebfd6-1d55-4487-84a5-b6a5a4ba1e47)
for instructions and the project rubric.

## Hints and Tips!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.
* Students have reported rapid expansion of log files when using the term 2 simulator.  This appears to be associated with not being connected to uWebSockets.  If this does occur,  please make sure you are conneted to uWebSockets. The following workaround may also be effective at preventing large log files.

    + create an empty log file
    + remove write permissions so that the simulator can't write to log
 * Please note that the ```Eigen``` library does not initialize ```VectorXd``` or ```MatrixXd``` objects with zeros upon creation.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! We'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Regardless of the IDE used, every submitted project must
still be compilable with cmake and make.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

