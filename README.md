# husky_ur3_simulator

## Overview
This is a mobile manipulator simulator package using Gazebo, RViz, MoveIt, move_base.

The model of the mobile manipulator robot was created by combining Universal Robots's UR3 and Clearpath Robotics's Husky.


### Author:
- **[TaeHyeon Kim](https://github.com/QualiaT), qualiatxr@gmail.com**
- **[Myunghyun Kim](https://github.com/kmh8667), kmh8667@khu.ac.kr**
- **[SungWoo Yang](https://github.com/Sungwwoo), p1112007@khu.ac.kr**

**Affiliation: [Human-Robot Interaction LAB](https://khu-hri.weebly.com), Kyung Hee Unviersity, South Korea**



## Installation

### Dependencies
This software is built on the Robotic Operating System ([ROS](http://wiki.ros.org/ROS/Installation)).

- For Husky mobile robot control & navigation install
```
$ sudo apt-get install ros-melodic-husky-description
$ sudo apt-get install ros-melodic-husky-gazebo
$ sudo apt-get install ros-melodic-husky-viz
$ sudo apt-get install ros-melodic-husky-navigation
```

- For [MoveIt](https://moveit.ros.org/) install
```
$ sudo apt-get install ros-melodic-moveit
```

- For [ar_track_alvar package](https://github.com/QualiaT/ar_track_alvar) install
```
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/QualiaT/ar_track_alvar.git
$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

### husky_ur3_simulator install
```
$ cd ~/catkin_ws/src
$ git clone -b melodic-devel https://github.com/QualiaT/husky_ur3_simulator.git

$ echo "export GAZEBO_MODEL_PATH=${HOME}/catkin_ws/src/husky_ur3_simulator/models" >> ~/.bashrc
$ source ~/.bashrc

$ cd ~/catkin_ws && catkin_make
$ rospack profile && rosstack profile
```

### Installation of dependencies for using gripper
```
- robotis_controller_msgs install
$ git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework-msgs.git

- dynamixel_sdk install
$ git clone -b melodic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git

- robotis_controller install
$ git clone https://github.com/ROBOTIS-GIT/ROBOTIS-Framework.git

- ros_controllers
$ sudo apt-get install ros-melodic-ros-controllers
```


## How to start?
```
- Bring up Gazebo with the robot model
$ roslaunch husky_ur3_gazebo husky_ur3_HRI_lab.launch

- Bring up MoveIt & RViz
$ roslaunch husky_ur3_gripper_moveit_config Omni_control.launch

- If you want to navigation using map, type the following command.
$ roslaunch husky_ur3_navigation husky_ur3_in_HRI_lab_amcl.launch
```

## How to use gripper?
```
- gripper open
$ rostopic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 0.0"

- gripper close
$ rostopic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 1.05"
```

## Demo
- Bring up Gazebo with the robot model
![01](https://user-images.githubusercontent.com/87522493/126894178-fff15a46-084b-467d-ab79-00342c11b3d9.png)

- Bring up RViz
![02](https://user-images.githubusercontent.com/87522493/126894179-931a6e86-1f23-4c39-a117-6e848778c900.png)

- Use navigation stack and control the husky_ur3 robot by 2D Nav Goal
![03](https://user-images.githubusercontent.com/87522493/126894180-eee58562-234c-4c83-94c1-aa34b27d8c7f.png)
![04](https://user-images.githubusercontent.com/87522493/126894175-82393fef-d536-472d-97c4-1f9745dc5dee.png)
![05](https://user-images.githubusercontent.com/87522493/126894176-69413f38-e58f-4528-adea-48e183a290ef.png)

- Using gripper
![06](https://user-images.githubusercontent.com/87522493/135451260-1c6f8d0a-3add-4038-a1b6-956e9d5c8c1c.png)
![07](https://user-images.githubusercontent.com/87522493/135451273-42ccdfc3-d666-4838-9202-be8769894c86.png)
