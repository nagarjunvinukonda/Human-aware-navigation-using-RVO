# Human-aware Navigation
Human-aware Navigation System for Robot.

**** NOTE **** This is a developing repository for our research. 

## Dependencies
This repository has been developed and tested in Ubuntu 18.04 and ROS Melodic only.

- The robot used in this package is `Gopher-Nurse-Robot`. 
  
  - `git clone git@github.com:hiro-wpi/Gopher-Nurse-Robot.git`
  - As we are only using the mobile base for navigation, there is no need to install the package of Konova robot arms. But remember to set the "with_arms" argument to "false" when launching the robot
  

#### Other packages needed:

- `ros_control`: ROS packages including controller interfaces, controller managers, transmissions, etc.
  - `sudo apt-get install ros-melodic-ros-control ros-melodic-ros-controllers`
  
- `gazebo_ros_pkgs` and `gazebo_ros_control`: Wrappers, tools and additional API's for using ROS with Gazebo
  
  - `sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control`
  
- `gmapping`, `amcl` and `move_base`: ROS SLAM, localization and navigation package
- `sudo apt-get install ros-melodic-map-server`
  - `sudo apt-get install ros-melodic-gmapping`
  - `sudo apt-get install ros-melodic-amcl`
  - `sudo apt-get install ros-melodic-move-base`
  - 
## Authors:
**Nagarjun Vinukonda and Zhuoyun Zhong**

## Running

- Launch the Gopher robot in Gazebo (Note the simulation is paused at the beginning.)

  `roslaunch gopher_gazebo spawn_gopher.launch`

  If you would like to launch the robot in any other worlds

  `roslaunch gopher_gazebo gopher.launch world_name:=<world_file_name>` 

