# Car Localization using Particle Filter
## Author : Sandeep Patil


[localization]: ./sample_images/localization.jpg "localization"

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![localization][localization]  

#### Objective of this project is to localization of car in map by using particle filter.   

In this project we will utilize particle filter method to locate car in map coordinate. 


--- 
### Installations and build 

This project involves the Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases) .

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. 

For Linux  
`./install-ubuntu.sh`

For Mac  
`./install-mac.sh`


Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./particle_filter

---

### Cummunication between main.cpp (server) and simulator
Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the x and y coordinate of available landmarks and car velocity and yaw_rate.  

OUTPUT: values provided by the c++ program to the simulator   

["position"] <= x and y coordinate of best particle.   
["yaw_anlge"] <= possible orientation of car   
["larmark"] <= landmark ids and their associated preicted measurements.   


---

### Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)


