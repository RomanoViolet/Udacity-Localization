# (Markov) Localization

## About
This project implements Markov Localization algorithm (EKF) as required by the Udacity's [Self Driving Car Nano-Degree program.](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)

The input to the algorithm are the position of a set of landmarks observed by the radar (or the lidar) mounted on the vehicle as it moves along a path. In addition, the vehicle is provided with a sparse map containing the position of a set of features. 
As the vehicle moves, the (noisy) sensors sense the location of objects in the vicinity, and the localization algorithm uses the data provided by the sensors, along with the map in order to precisely localize the vehicle.
The localization is probabilistic in nature, and is based on particle filters.
A rough estimate of the initial position of the vehicle is provided by a sensor (e.g., GPS). Thereafter, the localization is based solely on radar (and/or lidar) sensor data, along with the information contained in the map.

### How it Looks
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases). Select "Kidnapped Vehicle" option.

A sample frame from grabbed as the vehicle localizes itself inside a simulator is shown ![below](https://github.com/RomanoViolet/Udacity-Localization/blob/master/Results/screenshot.png)
The green trace is the path estimated by the Unscented Kalman Filter.

### Prerequisites
_Ad-verbatim from Udacity's Instructions:_

uWebSocketIO Starter Guide

All of the projects in Term 2 and some in Term 3 involve using an open source package called [uWebSocketIO](https://github.com/uNetworking/uWebSockets). This package facilitates the same connection between the simulator and code that was used in the Term 1 Behavioral Cloning Project, but now with C++. The package does this by setting up a web socket server connection from the C++ program to the simulator, which acts as the host. In the project repository there are two scripts for installing uWebSocketIO - one for Linux and the other for macOS.

Note: Only uWebSocketIO branch e94b6e1, which the scripts reference, is compatible with the package installation.
Linux Installation:

From the project repository directory run the script: install-ubuntu.sh


### Structure of the Project
The project is structured as follows:
- Folder "src": Contains the core logic required to build the localizer based on particle filters
- Folder "Results": Contains a sample video and a screenshot when testing the localizer using the simulator supplied by Udacity.
- `clean.sh`: A shell script to delete an old build (if it exists)
- `build.sh`: A shell script to build the Localizer project.
- `install-ubuntu.sh`: A script required for installing dependencies for running the simulator. See the section above on "Prerequisites"
- `run.sh`: A shell script to run the binary built using `build.sh`, and connects to the Udacity's Term 2 simulator. See the section above "How it Looks".


### Running the Localizer
Execute the `run.sh` which makes the Localizer application wait on the Udacity's Term 2 simulator. When running the simulator, choose the option "Kidnapped Vehicle".


### Pending Improvements
- Improvements to the accuracy of the Localizer Filter are possible, and are welcome.

### Credits
- Udacity: Lecturers, and mentors;
- Internet: for examples and samples.

### Disclaimer
Some of the ideas are borrowed and adapted from other people's work.
