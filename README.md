# ROS2-OpenCV project with UFACTORY LITE 6 ROBOTIC ARM
The purpose of this project is to apply the knowledge and experiences gained during the internship period collectively as a group (intern group) on a project.
Within the scope of this project, there are tasks such as utilizing the foundations of ROS2 and OpenCV for a robotic arm to detect boxes of different colors, pick them up, and align them to a specific location.
Interns who contributed: [Meriç SETAN](https://github.com/MericSetan) , [Yaren ÇOLAK](https://github.com/yarencolak) , [Esma Nur KARAKUŞ](https://github.com/esmaakarakuus)

# Requirements and Tested With

- Ubuntu-22.04
- ROS2-Humble
- [xarm_ros2](https://github.com/xArm-Developer/xarm_ros2.git)
- UFACTORY LITE 6 ROBOTIC ARM

# Installation

1) Install xarm_ros2(Humble) from https://github.com/xArm-Developer/xarm_ros2/tree/humble
2) Edit xarm_api config file {YOUR_WROKSPACE}/src/xarm_ros2/xarm_api/config/xarm_params.yaml
```
 services:
      debug: true  # When debug is true, all services will be turned on
```
3) 
```
cd {YOUR_WROKSPACE}/src/
git clone https://github.com/MericSetan/lite6_arm_ROS2_OpenCv_example.git
cd ..
colcon build --symlink-install
cd 
source .bashrc
```

# Usage

Setup [UFACTORY LITE 6 ROBOTIC ARM](https://manuals.plus/ufactory/lite-6-robotic-arm-manual#axzz8BNmzAvwo)

After establishing the connection between the computer and the robot using an Ethernet: (You can test on http://192.168.1.169:18333/ )


- Start the robot driver and xarm_api
```
ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.1.169 
```
- Start a server node that is ready to detect the boxes. (Requires a USB camera and parameter adjustments!!)

```
ros2 run lite6_arm find_boxes_server
```

- Start the Robot Control Node
```
ros2 run lite6_arm robot_control
```

! Inside the robot control node: There are clients created for the services and actions provided by the robot. Additionally, the main loop is contained within this node. In short, it includes the services, topics, actions, and parameters required for box detection and robot movement.
 
