# ukf_localization_movable_cam


## Introduction

This repository contains a set of MATLAB scripts simulating UKF (Unscented Kalman Filter)-based
self-localization of a vehicle. The vehicle model is based on a 4-wheeled e-bike, whose front steering rod is
movable similar to motorcyles or bicycles.

UKF here 
Information necessary for 'localizing' or estimating the vehicle's position comes
from the encoder attached to the wheel, the angular sensor attached to the steering rod,
and landmark image captured by a camera.

The scripts provides three types of self-localization system: Method A, B and C.

- Method A

  Camera is attached to the steering rod. 
  Since camera's pose relative to the vehicle's origin is contaminated with mechanical noise,
  localization data based on landmark image is contaminated as well.
  In this method , error covariance matrices given to UKF is calculated considering
  this noise.
  
- Method B

  Camera is attached to the steering rod. 
  In this method , error covariance matrices given to UKF is calculated ignoring
  mechanical noise of the steering rod relative to vehicle's origin.
  
- Method C

  Camera is attached to the vehicle's main frame.. 
  In this method,camera's pose relative to vehicle's main frame is constant.


## Prerequisite

  MathWorks MATLAB R2015a or above
 
### Tested Environment

  - Software : MathWorks MATLAB R2015a (8.5.0.197613) 64-bit (win64)
  - OS : Microsoft Windows7 Professional
  - Hardware : EPSON Endeavor NJ5700
  - CPU: Intel Core i7-3740QM @ 2.70GHz
  - RAM: 16GB

  
# How to use

  - Download the scripts 
  
    ~~~~
    > (chdir to your work folder)
	> git clone https://github.com/Hideyasu-Miyagi/ukf_localization_movable_cam.git
    ~~~~
  
  - Invoke MATLAB

    Invoke MATLAB by clicking the icon corresponding to that software.
  
  - Execute following commands in MATLAB's command window
  
    ~~~~
	>> chdir "(your work folder)/ukf_localization_movable_cam"
	
	>> pwd
	
	>> IEEJ_paper2020_gensimrec
	(the script invokes simulation of Method A, Method B and Method C and generates figures used in the paper).
	
	>> IEEJ_paper2020_calc_errval
	(information on estimation error will be displayed)
    ~~~~



  

