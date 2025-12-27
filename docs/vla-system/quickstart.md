# VLA System Quick Start Guide

Get started with the Vision-Language-Action (VLA) system quickly with this guide.

## Prerequisites

Before starting with the VLA system, ensure you have:
- Ubuntu 22.04 or later
- ROS 2 Humble Hawksbill installed
- Python 3.10 or later
- Docker (optional, for isolated development)

## Installation

### 1. Clone the repository
```bash
git clone <repository-url>
cd physical-ai-humanoid-robotics
```

### 2. Install system dependencies
```bash
sudo apt update
sudo apt install python3-dev python3-pip ros-humble-desktop
```

### 3. Create a virtual environment and install Python packages
```bash
python3 -m venv vla_env
source vla_env/bin/activate
pip install --upgrade pip

# Install ROS2 Python packages
pip install rclpy

# Install Whisper and Hugging Face transformers
pip install openai-whisper transformers torch

# Install other dependencies
pip install numpy opencv-python
```

### 4. Build the ROS2 workspace
```bash
cd ~/humanoid_ws  # or your workspace directory
colcon build --packages-select vla_system
source install/setup.bash
```

## Running the System

### 1. Launch the complete VLA system
```bash
source ~/humanoid_ws/install/setup.bash
ros2 launch vla_system vla_system.launch.py
```

### 2. Test with voice commands
Once the system is running, you can issue voice commands to the robot. The system will:
- Recognize your speech using Whisper
- Plan actions using the LLM
- Execute actions on the robot

## Running Individual Components

You can also run individual components for development and testing:

### Voice Recognition Only
```bash
ros2 run vla_system whisper_node
```

### LLM Planning Only
```bash
ros2 run vla_system llm_planner
```

### Navigation Only
```bash
ros2 run vla_system navigation_node
```

### Perception Only
```bash
ros2 run vla_system perception_node
```

## Testing the System

To verify the system is working correctly:

1. Run the full system with `ros2 launch vla_system vla_system.launch.py`
2. Issue a simple command like "move forward 1 meter" or "find the red block"
3. Observe the robot's response in simulation or on hardware

## Troubleshooting

### Audio Input Issues
- Ensure microphone is connected and recognized by the system:
```bash
arecord -l
```
- Check that audio permissions are granted to the application

### LLM Performance Issues
- Ensure sufficient RAM is available for the model
- Consider using quantized models for slower systems
- Check internet connectivity if using hosted models

### ROS2 Communication Issues
- Verify all nodes are running:
```bash
ros2 node list
```
- Check topic connections:
```bash
ros2 topic list
ros2 topic info <topic_name>
```

## Next Steps

After getting the system running, you can:
1. Explore the different configuration parameters in `config/params.yaml`
2. Modify the prompt templates in the LLM planner
3. Experiment with different Whisper model sizes
4. Test with various types of commands