#  ROS 2 + Gazebo Harmonic Integration - Position Publisher Package

This repository contains a ROS 2 Python package named `pos_py_pkg`, designed to demonstrate communication between ROS 2 Jazzy Jalisco and the Gazebo Harmonic simulation environment.

> ✅ Compatible with:  
> - **Operating System**: Ubuntu 24.04 LTS  
> - **ROS 2 Distribution**: Jazzy Jalisco  
> - **Gazebo Version**: Harmonic  
> - 📚 Official Docs: [https://gazebosim.org/docs/harmonic/ros_installation/](https://gazebosim.org/docs/harmonic/ros_installation/)


## 📁 Project Structure
```
ros2-gazebo-pos-publisher/
└── pos_py_ws/
└── src/
└── pos_py_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
├── test/
└── pos_py_pkg/
└── publisher.py # Your ROS 2 node goes here
```

## 🛠️ Installation & Setup

### 1. Source ROS 2 Jazzy Environment
```
source /opt/ros/jazzy/setup.bash
```
## 2. Install Required ROS 2 & Gazebo Packages
```
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-ros-gz
sudo apt-get install ros-jazzy-tf-transformations
```
## 🧪 Launch Sample Gazebo World
```
source /opt/ros/jazzy/setup.bash
ros2 launch ros_gz_sim_demos diff_drive.launch.py
```
Leave the GUI window open while it is running.

## 🧱 Create Your ROS 2 Python Package
```
cd ~
mkdir -p ~/pos_py_ws/src
cd ~/pos_py_ws/src
ros2 pkg create --build-type ament_python pos_py_pkg
```
## 🔧 Install Dependencies
```
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
```
You should see:

#All required rosdeps installed successfully
## 🔨 Build the Package
```
cd ~/pos_py_ws
colcon build --packages-select pos_py_pkg
```
## ▶️ Run the Node
Open a new terminal:
```
source ~/pos_py_ws/install/setup.bash
source /opt/ros/jazzy/setup.bash
ros2 run pos_py_pkg publisher
```
If the blue robot starts moving in Gazebo, everything is working correctly.

## 📌 System Requirements
✅ Ubuntu 24.04 LTS (64-bit)  
✅ ROS 2 Jazzy Jalisco fully installed and sourced  
✅ Gazebo Harmonic properly installed and tested  
🧠 Basic knowledge of ROS 2 workspaces and nodes  
