# ROS2 Gazebo Position Publisher Package

This repository contains a ROS2 Python package (`pos_py_pkg`) developed for Gazebo simulation testing.

---

## 📁 Project Structure

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

---

## 🚀 How to Build and Run

Make sure you have ROS2 Jazzy Jalisco installed and sourced.

```bash
# Navigate to workspace
cd pos_py_ws

# Build the package
colcon build --packages-select pos_py_pkg

# Source the workspace
source install/setup.bash

# Run your node (replace <node_name> with actual node)
ros2 run pos_py_pkg <node_name>
