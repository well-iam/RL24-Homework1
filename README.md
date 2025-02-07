# Build your Robot Manipulator (Homework1)

## :package: About
Goals:
- Set a Rviz Visualization of the Robot using the URDF model of the robotic arm.
- Optimize Collision meshes to improve simulation efficiency.
- Install Controllers, Sensors and spawn the robot in the Gazebo Simulator.
- Add a camera to the Robot and initialize the relative plugin
- Control the Robot's joint positions using the position controller

Tests and results are included in the report file, running controllers and verifying the robot's behavior using rqt_graph. 
This package contains the developed code for the first homework of the Robotics Lab 2024/25 Course. The authors of the package are:
William Notaro, Chiara Panagrosso, Salvatore Piccolo, Roberto Rocco

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace.  If you want to only clone the content files without creating the repo folder, use
```
$ git clone https://github.com/well-iam/RL24-RepoHomework1
```
Build the three packages
```
$ colcon build --packages-select arm_description arm_gazebo arm_control
```
Source the setup files
```
$ source ~/ros2_ws/install/setup.bash
```

## :white_check_mark: Usage
Run the RVIZ environment
```
$ ros2 launch arm_description display.launch.py
```
To set the desired joint configuration, modify the 'command_msg.data' in the 'timer_callback' of the 'arm_controller_node.cpp' file inside the 'arm_control' package. Open another terminal, source the setup files, and then run the Gazebo environment:
```
$ ros2 launch arm_gazebo arm_gazebo.launch.py
```

## :warning: Warning
During the launch of the Gazebo environment you could face a list of errors similar to:
```diff
- [GUI] [Err] [SystemPaths.cc:378] Unable to find file with URI [model://arm_description/meshes/base_link.stl]
```
This is caused by the unsuccessful update of the GZ_SIM_RESOURCE_PATH environment variable. To fix the error use:
```
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/ros2_ws/install/arm_description/share
```
