# Homework1

## :package: About

This package contains the developed code for the first homework of the Robotics Lab 2024/25 Course.

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder, use
```
$ git clone https://github.com/well-iam/RL-RepoHomework1 .
```
Build the three packages
```
$ colcon build --packages-select arm_description arm_gazebo arm_control
```
Source the setup files
```
$ . install/setup.bash
```

## :white_check_mark: Usage
Run the RVIZ environment
```
$ ros2 launch arm_description display.launch.py
```
To 
Open another terminal, source the setup files, and then run the Gazebo environment:
```
$ ros2 launch arm_gazebo arm_gazebo.launch.py
```

## :warning: Warning
During the launch of the Gazebo environment you could face a list of errors similar to:
```
- [GUI] [Err] [SystemPaths.cc:378] Unable to find file with URI [model://arm_description/meshes/base_link.stl]
```
This is caused by the unsuccessful update of the GZ_SIM_RESOURCE_PATH environment variable. To fix the error use:
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/arm_description/share
```
