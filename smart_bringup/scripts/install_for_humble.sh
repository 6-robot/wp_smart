#!/bin/bash
set -e

sudo apt update -y
sudo apt install -y ros-humble-slam-toolbox
sudo apt install -y ros-humble-teleop-twist-keyboard
sudo apt install -y ros-humble-navigation2
sudo apt install -y ros-humble-nav2-bringup
sudo apt install -y ros-humble-usb-cam
sudo apt install -y ros-humble-ros2-control
sudo apt install -y ros-humble-ros2-controllers
sudo apt install -y ros-humble-xacro
sudo apt install -y ros-humble-robot-state-publisher
sudo apt install -y ros-humble-joint-state-publisher
sudo apt install -y ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-nav2-msgs ros-humble-rosidl-default-generators
sudo apt install -y qtbase5-dev libyaml-cpp-dev libtinyxml2-dev ros-humble-tinyxml2-vendor
sudo apt install -y ros-dev-tools
sudo apt install -y python3-pip
sudo apt install -y python3-colcon-common-extensions
sudo apt install -y python3-rosdep python3-serial python3-yaml git
sudo apt install -y python3-argcomplete
sudo apt install -y pcl-tools
