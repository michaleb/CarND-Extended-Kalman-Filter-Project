## Extended Kalman Filter


[//]: # (Image References)

[image1]: ./IMG/Sensor_Fusion.png "Sensor Fusion flow diagram"
[image2]: ./IMG/RMSE-values.png "RMSE of predictions"
[image3]: ./IMG/radar.jpg "Radar measurements"
[image4]: ./IMG/kalman-filter-equ.jpg "Kalman Filter equations"
[image5]: ./IMG/radar-measurement-func.jpg "Radar Measurement functions"


### Introduction

This project implements the extended Kalman filter in C++. It uses a Kalman filter, lidar and radar measurements to track a bicycle's position and velocity. A linear motion model is employed, where acceleration is assumed to be zero, to predict the transition of the object's state (position, velocity). Sensor measurements are then used to improve upon our a priori prediction. For lidar its measurement of position are linear, in the case of non-linear radar measurements a Jacobian matrix is used to provide a linear approximation of the partial derivatives of its measured values (rho, phi, rho dot) with respect to (px, py, vx, vy) for 2-D motion.

The predict and update equations of the Kalman filter are shown below.
### Kalman Filter equations
![alt text][image4]


### Radar measurements

Whereas radar has three measurements (rho, phi, rho dot) and its prediction and measurement functions are both non-linear, lidar prediction and measurement functions are both linear. These measurement values along with their timestamp will be fed into the Kalman filter algorithm. The measurement function used to transform the predicted state into the measurement space will depend on the type of sensor being processed at that time. However, the prediction function remains the same throughout as a constant velocity model is assumed. 

![alt text][image3] ![alt text][image5]



### Process Flow

![alt text][image1]

The Kalman Filter algorithm will go through the following steps:

#### *first measurement* 
 - The filter receives initial measurements of the bicycle's position relative to the car. These measurements will come from a radar or lidar sensor.

 Every time main.cpp calls fusionEKF.ProcessMeasurement(measurement_pack_list[k]), the code in FusionEKF.cpp will run. If this is the first measurement, the Kalman filter will try to initialize the object's location with the sensor measurement.

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

#### *initialize state and covariance matrices*
 - The filter initializes the bicycle's position based on the first measurement. The car will then receive another sensor measurement after a time period Δt.

 ```cpp
    // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.Q_ = MatrixXd(4, 4);
  
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // measurement matrix
  H_laser_ <<  1, 0, 0, 0,
               0, 1, 0, 0;

  // the initial transition matrix F_
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // state covariance matrix P
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

```
#### *predict*
 - The algorithm predicts where the bicycle will be after time Δt. It is assumed that there are no changes in the bicycle's velocity; thus the bicycle will have moved a distance of *(velocity * Δt)*. 

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
```  
```cpp
void KalmanFilter::Predict() {
  
  MatrixXd F_t = F_.transpose();
  x_ = F_ * x_;
  P_ = F_ * P_ * F_t + Q_;
} 
//The prediction for Radar is made in cartesian space, however the update is in polar space...hence must 
//convert back to polar prior to update
```


#### *update*
 - The filter compares the "predicted" location with the sensor's measurement. The predicted location and the measured location are combined to give an updated location. The Kalman filter places more weight on either the predicted location or the measured location depending on the uncertainty of each value. 


```cpp
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

Update method for linear LIDAR measurements...
```cpp

void KalmanFilter::Update(const VectorXd &z) {
  
  VectorXd y = z - H_ * x_;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * H_t * Si;

  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
```

Update method for non-linear RADAR sensor measureents...
```cpp
void KalmanFilter::UpdateEKF(const VectorXd &z) {
  
  //Convert predicted state of x_ from Cartesian to polar coordinates
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float rho = pow(pow(px,2) + pow(py,2),0.5);
  float phi = atan2(py, px);
  float rho_dot;

  // check division by zero
  if (fabs(rho) < 0.0001) { 
    rho_dot = 0;
  }
  else {
    rho_dot = (px*vx + py*vy)/rho;
  }

  //x_prime a 3x1 vector is used to store polar equivalent of predicted state x_
  //which can then be compared with the radar measurement z also a 3x1 vector to find the error, y,
  // between prediction and actual measurement in the current update.

  VectorXd x_prime(3);
  x_prime << rho,
             phi,
             rho_dot;  

  VectorXd y = z - x_prime;
  
  //adjust phi to range -pi to +pi
  if (y(1) < -M_PI) {
    y(1) += 2*M_PI;
  } 
  else if (y(1) > M_PI) {
    y(1) -= 2*M_PI;
  }
  
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * H_t * Si;           
  
  // new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
```
The car then receives another sensor measurement after a time period Δt and once again the algorithm does a predict and update step.

### Result of Kalman filter estimations
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

