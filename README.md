# 🤖 Surgical Tools Sorter 🤖

> Authors:
> - [Anirudh Swarankar](aniswa@umd.edu)
> - [Munyaradzi Antony](mantony2@umd.edu)
> - [Pranav Deshakulkarni Manjunath](pranavdm99@umd.edu)
> - [Varad Nerlekar](nerlekar@umd.edu)


## 📚 Table of Contents
- [Introduction](#introduction)
    - [Problem Statement](#problem-statement)
- [Solution Overview](#solution-overview)
- [Package Structure](#package-structure)
- [ROS2 Communication](#ros2-communication)
    - [Topics](#topics)
    - [Actions](#services)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)


## 🚀 Introduction
### 💡 Problem Statement
Design a system for sorting surgical tools using a 6-DOF Robot Arm, enabling precise identification, manipulation, and placement of various surgical tools in a sterile environment.

## 📖 Background
- **Enhanced Surgical Efficiency**: Automate tool sorting, ensuring that precise organization and quick accessibility of surgical tools, reducing the time spent by medical staff in tool retrieval during procedures.
- **Minimized Human Error**: Reduce the risk of human error in misplacing or misidentifying instruments, contributing to a safer surgical environment.
- **Sterility Maintenance**: Operations in sterile environments, minimizing human contact with tools and maintaining hygiene and safety standards during surgery.
- **Integration with Smart Systems**: The solution can be integrated with surgical management systems for real-time tracking of tools, ensuring accurate inventory management and reducing the risk of tool shortages or mismatches.
- **Application in High-Volume Facilities**: In large hospitals or surgical centers, robotic sorting systems can handle the high volume of tools required for various procedures, enabling faster preparation and turnover between surgeries.


## ⚡️ Solution Overview
The Surgical Tools Sorter utilizes a 6-DOF robotic arm equipped with advanced perception capabilities to identify and sort surgical tools. The system employs computer vision techniques to detect and classify tools, followed by precise manipulation using the robotic arm to place them in designated locations. This automation enhances the efficiency and safety of surgical environments.

## 📦 Package Structure
```
├── inverse_kinematics
│   ├── inverse_kinematics
│   │   ├── dummy_pub.py
│   │   ├── __init__.py
│   │   └── inverse_kinematics.py
│   ├── launch
│   │   └── inverse_kinematics_launch.py
│   ├── package.xml
│   ├── README.md
│   ├── resource
│   │   └── inverse_kinematics
│   ├── setup.cfg
│   └── setup.py
├── inverse_kinematics_interfaces
│   ├── action
│   │   └── Posegoal.action
│   ├── CMakeLists.txt
│   ├── msg
│   │   └── JointAngles.msg
│   ├── package.xml
│   └── README.md
├── LICENSE
├── README.md
├── requirements.py
└── surgical_tools_sorter
    ├── CMakeLists.txt
    ├── launch
    │   └── ign_gazebo.launch.py
    ├── models
    │   ├── robot_arm
    │   │   ├── base_link
    │   │   │   └── base_link.STL
    │   │   ├── camera_link
    │   │   │   ├── camera_link.STL
    │   │   │   └── camera.stl
    │   │   ├── eb100.sdf
    │   │   ├── gripper_left_link
    │   │   │   └── gripper_left_link.STL
    │   │   ├── gripper_right_link
    │   │   │   └── gripper_right_link.STL
    │   │   ├── link_1
    │   │   │   └── link1.STL
    │   │   ├── link_2
    │   │   │   └── link2.STL
    │   │   ├── link_3
    │   │   │   └── link3.STL
    │   │   ├── link_4
    │   │   │   └── link4.STL
    │   │   ├── link_5
    │   │   │   └── link5.STL
    │   │   └── link_6
    │   │       └── link6.STL
    │   ├── scalpel
    │   │   ├── meshes
    │   │   │   ├── o_scalpel_v2.obj
    │   │   │   └── scalpel_v2.mtl
    │   │   └── scalpel.sdf
    │   ├── scissors
    │   │   ├── meshes
    │   │   │   ├── o_scissors_v3.obj
    │   │   │   └── scissors_v3.mtl
    │   │   └── scissors.sdf
    │   ├── syringe
    │   │   ├── meshes
    │   │   │   ├── o_syringe_part_v3.obj
    │   │   │   └── syringe_part_v3.mtl
    │   │   └── syringe.sdf
    │   ├── table
    │   │   ├── meshes
    │   │   │   ├── o_table_link.obj
    │   │   │   ├── table_link.mtl
    │   │   │   └── table_link.STL
    │   │   └── table_link.sdf
    │   ├── tray
    │   │   ├── meshes
    │   │   │   ├── o_tray_link_v2.obj
    │   │   │   └── tray_link_v2.mtl
    │   │   └── tray_link.sdf
    │   ├── tweezer
    │   │   ├── meshes
    │   │   │   ├── o_tweezers_part_v3.obj
    │   │   │   └── tweezers_part_v3.mtl
    │   │   └── tweezer.sdf
    │   └── vaccine_bottle
    │       ├── ibuprofen.sdf
    │       ├── meshes
    │       │   ├── ibuprofen.mtl
    │       │   ├── o_ibuprofen.obj
    │       │   ├── o_propofol.obj
    │       │   └── propofol.mtl
    │       └── propofol.sdf
    ├── package.xml
    ├── scripts
    │   └── perception
    │       ├── __init__.py
    │       ├── model
    │       └── tools_in_tray.py
    └── worlds
        ├── just_arm.sdf
        └── tool_sorter_world.sdf

39 directories, 61 files
```

## 📝 ROS2 Communication
### Topics
- `/camera/image`: Camera feed for tool detection.
- `/joint_angles/joint1`: Joint angles for controlling the joint 1 of the robotic arm.
- `/joint_angles/joint2`: Joint angles for controlling the joint 2 of the robotic arm.
- `/joint_angles/joint3`: Joint angles for controlling the joint 3 of the robotic arm.
- `/joint_angles/joint4`: Joint angles for controlling the joint 4 of the robotic arm.
- `/joint_angles/joint5`: Joint angles for controlling the joint 5 of the robotic arm.
- `/joint_angles/joint6`: Joint angles for controlling the joint 6 of the robotic arm.
- `/joint_angles/gripper_left_joint`: Joint angle for the left gripper.
- `/joint_angles/gripper_right_joint`: Joint angle for the right gripper.

### Actions
- `/goal`: Action to move the robotic arm to a specified pose for tool manipulation.

## 🧰 Requirements
- Ubuntu 22.04 (or WSL/Docker running Ubuntu 22.04)
- ROS2 Humble
- Ignition Gazebo (Fortress)

## 👨‍💻 Installation
To install the Surgical Tools Sorter package, follow these steps:
1. Create a ROS2 workspace:
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Clone the repository:
```
git clone https://github.com/Arthav24/surgicalBot.git
cd ~/ros2_ws
```

3. Build the packages and source them:
```
colcon build --symlink-install && source install/setup.bash
```

## 💻 Usage
To launch the Surgical Tools Sorter Environment, use the following command:
```
ros2 launch surgical_tools_sorter ign_gazebo.launch.py
```

To launch the Inverse Kinematics Node, use the following command:
```
ros2 launch inverse_kinematics inverse_kinematics.launch.py
```

And finally to test the Robot Arm motion, use the following command:
```
ros2 action send_goal /goal inverse_kinematics_interfaces/action/Posegoal "{pose: {position: {x: <user-input>,y: <user-input>,z: <user-input>}, orientation: {w: <user-input>}}}" --feedback
```

## 📃 License
This project is licensed under the Apache License. See the LICENSE file for details.

Feel free to adjust any sections or add more details as necessary to better fit your project's specifics!
