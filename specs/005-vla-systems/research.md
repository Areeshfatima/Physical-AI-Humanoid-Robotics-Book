# Research for Vision-Language-Action Systems

## Decision: Whisper Module Size Choice
**Rationale**: For educational purposes and to run efficiently on standard hardware, we'll use the "base" model which provides a good balance between performance and resource requirements. The "base" model is approximately 145MB compared to "large" at 3.0GB, while still providing good accuracy for speech recognition tasks.
**Alternatives considered**: 
- "tiny" model (75MB) - too inaccurate for educational purposes
- "small" model (484MB) - good accuracy but requires more resources than necessary
- "medium" model (1.5GB) - good accuracy but may strain educational hardware
- "large" model (3.0GB) - excellent accuracy but too resource-intensive for educational use

## Decision: LLM Provider
**Rationale**: Using Hugging Face transformers with open-source models (like Llama 2/3, Mistral, or similar) allows students to understand and modify the underlying models without licensing concerns, aligning with the educational goals of the project. These models can run locally and provide good performance for planning tasks.
**Alternatives considered**:
- OpenAI API - requires internet connection and subscription costs
- Anthropic API - similar issues as OpenAI API
- Custom trained models - too complex for educational implementation
- Local models (like Ollama) - option but Hugging Face provides better educational insights

## Decision: Planning Strategy (Direct Mapping vs Hierarchical)
**Rationale**: A hierarchical planning approach is chosen for the educational value and real-world applicability. This allows students to understand how complex tasks are broken down into subtasks, which is important for robotics applications. The hierarchical approach also provides better error handling and recovery options.
**Alternatives considered**:
- Direct mapping - simpler but less educational value
- State machine - good for simple tasks but not flexible enough for complex planning
- Behavior trees - good for robotics but less generalizable to other planning problems

## Decision: ROS2 Action Graph Design
**Rationale**: Using ROS 2 action servers and clients for the action execution component provides the best approach for long-running robot behaviors with feedback. This allows for proper handling of navigation and manipulation tasks that require continuous feedback and status updates.
**Alternatives considered**:
- Simple topics - insufficient for tracking long-running tasks
- Services - synchronous and not appropriate for continuous robot behaviors
- Custom action servers - reinventing existing functionality

## Decision: Integration Architecture
**Rationale**: A node-based architecture where each major component (Whisper, LLM, Navigation, Perception) runs as a separate ROS 2 node provides the best modularity and educational value. This allows students to understand and modify individual components independently.
**Alternatives considered**:
- Monolithic approach - harder to understand and debug
- Service-based architecture - less suitable for continuous robot behaviors

## Technology Research Findings

### Whisper Implementation
- OpenAI Whisper is available via Hugging Face transformers
- Requires minimal setup for inference
- Works well with audio input from microphone or file
- Supports multiple languages but for educational purposes, focusing on English

### LLM Cognitive Planning
- Hugging Face transformers provide access to various open-source LLMs
- Models like Llama 2, Llama 3, Mistral, or Phi-3 are suitable for planning tasks
- Requires careful prompt engineering to generate proper ROS2 action sequences
- Local inference allows for experimentation without API costs

### ROS 2 Integration
- Using rclpy for Python-based ROS 2 nodes
- Action servers for long-running tasks like navigation
- Services for short, synchronous operations
- Topics for continuous sensor data and state updates
- Using Nav2 for navigation stack integration

### Performance Optimization
- Whisper "base" model can run on standard educational hardware
- Quantized LLM models can reduce resource requirements
- Proper buffering of audio input for real-time processing
- Efficient state management to prevent re-planning when not needed

## Architecture Sketch

```
[Voice Input] -> [Whisper Node] -> [Transcribed Text]
                      |                    |
                      v                    v
              [LLM Planner Node] -> [Action Plan]
                      |                    |
                      v                    v
         [VLA Manager] -> [Action Execution] -> [Robot Actions]
                            |
                            v
              [Perception] <- [Navigation] -> [Robot State]
```

This architecture allows each component to be tested independently while providing the complete VLA pipeline for end-to-end functionality.