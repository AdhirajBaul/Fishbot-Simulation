# Fishbot

A fish-inspired robot simulation built using ROS2 Humble and Gazebo Classic.

## Current Features

- Fish robot modeled in URDF
- Tail fin and pectoral fins
- Ocean world simulation
- RViz visualization
- Gazebo simulation
- ros2_control integration (in progress)

## Software Stack

- ROS2 Humble
- Gazebo Classic 11
- Ubuntu 22.04
- WSL2
- Python

## Project Structure

fishbot_ws/
├── src/
│   ├── fishbot_description/
│   └── fishbot_control/

## Run

```bash
source install/setup.bash
ros2 launch fishbot_description gazebo.launch.py