# Robotics : GO Chase It!
A robot chases a white ball. The program runs in the robot simulation [Gazebo](http://gazebosim.org/). The software is programmed in C++ and that uses the Robot Operating System [ROS]( https://www.ros.org/).    
    
This project is part of my robotics nano degree of[ Udacity](https://www.udacity.com/course/robotics-software-engineer--nd209). 

    
### Examples

+ **A robot chases a white ball**
<img src="go_chase_it_video.gif" width="80%" height="80%" />   
   
   
   
## Important files
- /my_world/ : the ROS package with the roboter simulation
- /ball_chaser/ : the ROS package with the source code
- /ball_chaser/**drive_bot.cpp** : the ROS node that drives the robot to the white ball
- /ball_chaser/src/**process_image.cpp** : the ROS node that subscribe to the robot’s camera images and analyze them to determine the position of the white ball
- /ball_chaser/srv/**DriveToTarget.srv**: the publishes messages containing the velocities for the wheel joints
- /ball_chaser/**CMakeLists.txt** : the catkin make file

   
### File structure
<img src="file_structure.PNG" width="60%" height="60%" /> 

    
## Installation and usage
Clone the repository
```sh
$ cd <your workspace folder>
$ git clone https://github.com/embmike/RobotND-Go-Chase-It.git
```
    
    
## Licence
This project is licensed under the terms of the [![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
