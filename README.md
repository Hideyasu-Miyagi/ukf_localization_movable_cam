## ukf_localization_movable_cam


# Introduction

This repository contains MATLAB scripts simulating UKF-based self-localization
of a vehicle. The vehicle is a 4-wheel e-bike, whose front steering rod is
movable similar to motorcyles or bicycles.

Information necessary for 'localizing' or estimating the vehicle's position comes
from the encoder attached to the wheel, the angular sensor attached to the steering rod,
and landmark image captured by a camera.


# Prerequisite

  MathWorks MATLAB R2015a or above
  
  
# How to use

  - Download the scripts 
  
    ~~~~
    (chdir to your work folder)
	git clone https://github.com/Hideyasu-Miyagi/ukf_localization_movable_cam.git
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



  

