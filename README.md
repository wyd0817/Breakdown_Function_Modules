# Breakdown_Function_Modules

A ROS2 package for multi-robot task decomposition and dependency management.

## Description

Breakdown_Function_Modules is an intelligent task management system designed to work within ROS2-based robotic frameworks. Working in conjunction with QA_LLM_Module, it specializes in parsing complex instructions, decomposing them into manageable subtasks, and managing dependencies for coordinated multi-robot operations.

## Key Features

- **Task Decomposition Engine**
  - Parses high-level instructions
  - Generates structured, executable commands
  - Optimizes task distribution

- **Dependency Management System**
  - Tracks inter-task dependencies
  - Ensures proper execution order
  - Manages resource allocation

- **ROS2 Integration**
  - Seamless communication protocol
  - Compatible with multi-robot architectures
  - Built-in message handling

## Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- QA_LLM_Module

## Installation

1. Clone the repository into your ROS2 workspace:
```bash
cd ~/your_ros2_workspace/src
git clone https://github.com/wyd0817/Breakdown_Function_Modules.git
```

2. Build the package:
```bash
cd ~/your_ros2_workspace
colcon build --packages-select breakdown_function_modules
```

3. Source the workspace:
```bash
source ~/your_ros2_workspace/install/setup.bash
```

## Usage

### Basic Operation

Launch the breakdown function node:
```bash
ros2 run breakdown_function_modules breakdown_function_node
```
