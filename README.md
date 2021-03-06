# SGM-SIM: a Soft Growing Manipulator SIMulator

This repository contains a soft growing manipulator simulation scenario built in [Coppelia Sim](http://coppeliarobotics.com "Coppelia Robotics Homepage"). The main simulation scene is developed for a shared control teleoperation study using a novint falcon haptic interface (code is provided). However it can be used standalone to develop your own soft growing manipulation application.

![alt text](sgm-simulation.gif)

### Reference 
If you use this code, please cite our paper: 
*F. Stroppa et al., "Shared-control Teleoperation Paradigms on a Soft Growing Manipulator", Submitted.*

# Preliminaries
## CoppeliaSim
Download Coppelia Sim from [this page](https://www.coppeliarobotics.com/downloads "Coppelia Robotics Download Page") or running the following command

`$ wget https://www.coppeliarobotics.com/files/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04.tar.xz`

Extract to a desired location, e.g.

`$ tar -xvf ~/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04.tar.xz`

Test your installation 

`$ cd ~/CoppeliaSim_Edu_V4_1_0_Ubuntu18_04; ./coppeliaSim.sh`

### Standalone use

If you want to use the soft growing manipulator simulation and develop your own application, download the scene *scenes/sgm-sim.ttt* and open it in Coppelia Sim.

### Shared control teleoperation

If you want to use our code to interface the simulation environment or test our shared control teleoperation architectures follow the steps below.

## Eigen
Eigen can be downloaded from [this page](http://eigen.tuxfamily.org/index.php?title=Main_Page#Download) or cloning its repo

`$ git clone https://gitlab.com/libeigen/eigen.git`

Install Eigen with the following command

`$ cd ~/eigen; mkdir build; cd build; cmake ..; sudo make install`

## Novint Falcon
Novint Falcon libraries are at [this page](https://github.com/libnifalcon/libnifalcon). To install them, clone the repo

`$ git clone https://github.com/libnifalcon/libnifalcon.git`

`$ cd libnifalcon`

and follow [these instructions](https://github.com/libnifalcon/libnifalcon/blob/master/COMPILE.txt). 

# Build and run

Download the source code or clone this repository with the command

`$ git clone https://github.com/mrslvg/sgm-sim.git ~/sgm-sim`

To build the source code

`$ cd ~/sgm-sim/shared-control; mkdir build; cd build; cmake ..; make -j8`

To run the shared control teleoperation, simply run

`$ ./shared-control`

### Note
Make sure the Coppelia Sim is open and the scene *sgm-sim.ttt* is correctly loaded before executing the last command.
