# Quickstart Guide: Vision-Language-Action Systems

## Prerequisites

- Ubuntu 22.04 or later
- ROS 2 Humble Hawksbill installed
- Python 3.10 or later
- Docker (optional, for isolated development)

## Setup

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

## Running the VLA System

### 1. Launch the complete system
```bash
source ~/humanoid_ws/install/setup.bash
ros2 launch vla_system vla_system.launch.py
```

### 2. Interact with the system
Once the system is running, you can:

- Speak commands to the microphone
- Or send text commands via the command line:
```bash
# Send a text command directly
ros2 service call /speak_command std_msgs/String "data: 'move forward 1 meter'"
```

### 3. Monitor system status
```bash
# View robot state
ros2 topic echo /robot_state

# View action plans
ros2 topic echo /action_plans

# View perception data
ros2 topic echo /perception_data
```

## Testing Individual Components

### Test Whisper Node
```bash
# Run just the speech recognition component
ros2 run vla_system whisper_node
```

### Test LLM Planner
```bash
# Run just the planning component
ros2 run vla_system llm_planner
```

### Test Navigation
```bash
# Run navigation component with a simulated robot
ros2 launch nav2_bringup tb3_simulation_launch.py
```

## Configuration

### Adjust Whisper Parameters
Edit `config/whisper_params.yaml` to adjust:
- Model size (tiny, base, small, medium, large)
- Language detection
- Processing sensitivity

### Adjust LLM Parameters
Edit `config/llm_params.yaml` to adjust:
- Model selection
- Prompt templates
- Response timeout

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

1. Follow the [detailed architecture guide](../docs/architecture.md) for a deeper understanding
2. Review the [API contracts](./contracts/openapi.yaml) for programmatic interaction
3. Set up the simulation environment to test without hardware
4. Customize the prompt templates for specific robot behaviors