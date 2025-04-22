# SGBA_CF2_App_layer

An application layer implementation of the Swarm Gradient Bug Algorithm (SGBA) for the Crazyflie 2.0 platform, accomendating the publication of:

> K.N. McGuire, C. De Wagter, K. Tuyls, H.J. Kappen, G.C.H.E. de Croon,
> 'Minimal navigation solution for a swarm of tiny flying robots to explore an unknown environment'
> Science Robotics, 23 October 2019 
> DOI: http://robotics.sciencemag.org/lookup/doi/10.1126/scirobotics.aaw9710


## Overview

This repository provides the application-level code for executing the Swarm Gradient Bug Algorithm (SGBA) on Crazyflie 2.0 drones. SGBA enables autonomous swarm behaviors, such as obstacle avoidance and goal-directed navigation, suitable for collaborative robotics applications.

## Features

- Implementation of SGBA tailored for Crazyflie 2.0
- Modular code structure for easy integration and testing
- Support for multi-agent coordination and communication (this p2p communication was disabled for our use case)
- Real-time obstacle avoidance and goal navigation

## Getting Started
### Installation
1. Create a workspace, git clone this repo
```
git clone https://github.com/TL-NUS-CFS/SGBA_CF2_App_layer.git

cd SGBA_CF2_App_layer 
```

2. Checkout Working Branch & Initialize Submodules
```
git checkout competition_known_SGBA
git submodule init
git submodule sync
git submodule update
```

### Flashing onto Crazyflie
1. Navigate into the Crazyflie firmware directory:
```cd crazyflie-firmware```
2. Run menuconfig: ```make menuconfig```
- Set WiFi SSID and password
- Enable App Layer configuration
    - App layer configuration > Enable entry point
3. Return and build
```
cd ..
make clean && make
```
4. Put Crazyflie into bootloader mode:
- Turn the drone off,  hold power button until blue LED blinks
5. Flash the firmware with Crazyradio plugged in: ```make cload``

## Important Branches
- competition_known_SGBA: SGBA code used for drones in Known Area
- competition_known_WF: Wall-following code used for drones in the Known Area
- competition_unknown_SGBA: SGBA coe used for drones in the Unknown Area
- competition_pillar_right: Not SGBA. Algorithm for Pillar Area drones

## Important files
- drone_variables.c: Contain flight parameters, like wall-following distances, drone speed, height, and algorithm parameters
- state_machine.c: Main loop, able to toggle between wall-following and SGBA controllers. Handles receiving and transmission of radio command.
- SGBA.c: Controller code to execute SGBA
