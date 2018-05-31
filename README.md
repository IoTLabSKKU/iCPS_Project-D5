# iCPS Project Division 5: Intelligent CPS Middleware for Smart Vehicles
## About the project
The goal of this project is to develop a terminal middleware for smart vehicles in various iCPS applications such as smart transportation, smart farm and smart factory.

The goal of 1st year development is to design the middleware architecture and component technologies for smart transportation. This repository is the demo system of component technologies for smart transportation.

The system is implemented based on the ROS framework (http://www.ros.org/).

This project is developed and maintained by Networked Smart Systems Lab (NSSLab, http://nsslab.pusan.ac.kr/), Pusan National University.

## Hardware specifications
1. Laptop MSI GE63VR (Ubuntu 16.04 and ROS Kinetic)
2. Samsung Galaxy S7 (Android 6.0.1)
3. Cohda Wireless MK4/MK5 OBU and RSU
4. 2-D LiDAR SICK LMS511
5. Ublox GPS application board C94-M8P

## Software specifications
The demo source code of middleware is in "/icps" folder.

The driver for 2-D LiDAR can be downloaded from https://github.com/pangfumin/lms511_laser

The driver for C94-M8P can be downloaded from https://github.com/KumarRobotics/ublox

## How to compile
Create a catkin workspace (http://wiki.ros.org/catkin/Tutorials)

Clone the "/icps" folder and copy to src folder of catkin workspace and build the catkin workspace.
