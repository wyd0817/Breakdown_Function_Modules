# Breakdown_Function_Modules

Breakdown_Function_Modules is a core component of the DART-LLM (Dependency-Aware Multi-Robot Task Decomposition and Execution) system. It consists of two main components: the Breakdown Function Parser and Handler, which work together to process structured task descriptions, manage dependencies between subtasks, and coordinate execution across multiple robots.

## Features

- **Breakdown Function Parser**
  - Parses structured JSON output from QA_LLM_Module
  - Extracts instruction functions, parameters, and dependencies
  - Validates task sequences and dependency relationships

- **Breakdown Function Handler**
  - Manages task execution based on dependency graph (DAG)
  - Coordinates with object map database for location information
  - Enables parallel execution of independent tasks

- **Supported Functions**
  - Navigation Functions:
    - Area avoidance for all/specific robots
    - Target area guidance
    - Area access permissions
    - Return-to-start commands
  - Robot-Specific Functions:
    - Excavator operations (digging, unloading)
    - Dump truck operations (loading, unloading)

## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/wyd0817/Breakdown_Function_Modules.git
   ```

2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select breakdown_function_modules
   ```

3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

## Usage

### Basic Operation

Launch the breakdown function nodes:
```bash
ros2 launch breakdown_function_modules breakdown_function.launch.py
```

### Input/Output Format

The module processes JSON input from QA_LLM_Module in the following format:
```json
{
    "instruction_function": {
        "name": "<breakdown function>",
        "dependencies": ["<dep 1>", "<dep 2>", "...", "<dep n>"]
    },
    "object_keywords": ["<key 1>", "<key 2>", "...", "<key n>"]
}
```

### ROS2 Topics

- Subscribes to:
  - `/qa_llm/task_description`: Receives task descriptions
  - `/object_map/updates`: Receives object location updates
  
- Publishes to:
  - `/robot_commands`: Sends commands to robots
  - `/task_status`: Reports execution status

## Configuration

The module can be configured through:

- `config/`: ROS2 parameters and configuration files
- `launch/`: Launch files for different scenarios
- `src/`: Core implementation of parser and handler nodes


