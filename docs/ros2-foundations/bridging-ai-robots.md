---
sidebar_position: 4
---

# Bridging AI Agents to Robots + URDF for Humanoids

## Introduction

Modern robotics increasingly relies on artificial intelligence to provide cognitive capabilities. This chapter explores how to bridge AI agents with robotic systems using ROS 2, and how to define humanoid robots using URDF (Unified Robot Description Format). Understanding these concepts is crucial for creating intelligent, humanoid robotic systems.

## Bridging AI Agents to Robotic Systems

### The AI-Robot Interface

Bridging AI agents to robots involves creating communication channels between high-level AI decision-making systems and low-level robot control systems. This interface typically includes:

- **Perception Interface**: How AI agents consume sensor data from robots
- **Action Interface**: How AI agents send commands to robot actuators
- **State Interface**: How AI agents maintain awareness of robot state
- **Goal Interface**: How AI agents specify tasks and objectives for robots

### ROS 2 as the Bridge

ROS 2 provides the ideal middleware for AI-robot bridging because:
- **Language Agnostic**: AI models can be written in Python, C++, or other languages
- **Flexible Communication**: Topics, services, and actions accommodate different interaction patterns
- **Rich Ecosystem**: Extensive libraries for AI, perception, and control
- **Real-time Capabilities**: Sufficient performance for real-time AI-robot interaction

### Architecture Patterns

#### 1. Centralized AI Control
- Single AI agent makes all decisions
- Direct communication between AI and all robot components
- Suitable for simpler tasks with predictable environments

#### 2. Distributed AI Control
- Multiple AI agents specialize in different functions (perception, planning, execution)
- Coordination through ROS 2 topics and services
- Better for complex tasks requiring specialization

#### 3. Hybrid Approach
- Combination of reactive controllers and AI decision-making
- AI handles high-level planning, reactive controllers handle low-level execution
- Balances performance and intelligence

## URDF: Unified Robot Description Format

### What is URDF?

URDF (Unified Robot Description Format) is an XML format used to describe robots in ROS. It defines:
- **Kinematic structure**: How robot parts are connected
- **Visual properties**: How robot appears in simulation
- **Collision properties**: How robot interacts with environment
- **Physical properties**: Mass, inertia, and other physical characteristics

### URDF Structure for Humanoids

A humanoid robot URDF typically includes:
- **Trunk**: Main body with sensors and computing units
- **Limbs**: Arms and legs with multiple joints
- **Actuators**: Motors and servos controlling joint movement
- **Sensors**: Cameras, IMUs, and other sensing equipment
- **End Effectors**: Hands/feet for manipulation and locomotion

### Basic URDF Components

#### Links
Links represent rigid bodies in the robot:
```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

#### Joints
Joints connect links and define their motion:
```xml
<joint name="base_to_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0 0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### URDF for Humanoid Robots

Humanoid robots require special considerations in URDF:

#### 1. Kinematic Chains
- **Leg chains**: From hip to foot for locomotion
- **Arm chains**: From shoulder to hand for manipulation
- **Spine chain**: From pelvis to head for posture

#### 2. Joint Limitations
- Human-like joint limits for realistic movement
- Range of motion constraints for safety
- Torque limitations for actuator protection

#### 3. Sensor Integration
- Camera mounting points for vision
- IMU placement for balance
- Force/torque sensors for interaction

### Example: Simple Humanoid URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Trunk -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_to_head" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

## AI-Robot Integration Patterns

### 1. Behavior Trees
Behavior trees provide a structured way to define robot behaviors controlled by AI:
- Hierarchical composition of actions
- Clear decision-making logic
- Easy to debug and modify

### 2. Finite State Machines
FSMs are useful for discrete robot behaviors:
- Well-defined states and transitions
- Predictable behavior patterns
- Good for safety-critical applications

### 3. Neural Network Controllers
Direct neural network integration for learned behaviors:
- End-to-end learning from perception to action
- Adaptive behavior through training
- Requires significant computational resources

## Practical Implementation

### Setting up AI-ROS Communication

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Twist
import numpy as np

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Subscribe to sensor data
        self.image_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)

        # Publisher for robot commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)

        # Timer for AI processing loop
        self.timer = self.create_timer(0.1, self.ai_processing_loop)

    def image_callback(self, msg):
        # Process image data for AI perception
        self.process_image_data(msg)

    def joint_callback(self, msg):
        # Update AI with robot state
        self.update_robot_state(msg)

    def ai_processing_loop(self):
        # Main AI decision-making logic
        ai_decision = self.make_decision()
        self.publish_command(ai_decision)
```

## Exercises

1. **URDF Creation**: Create a simple humanoid robot URDF with trunk, head, and arms
2. **AI Bridge**: Implement a simple AI node that subscribes to sensor data and publishes commands
3. **Behavior Tree**: Design a behavior tree for a humanoid robot performing a simple task
4. **Simulation Integration**: Load your URDF into a ROS 2 simulation environment

## Summary

This chapter covered the essential concepts of bridging AI agents to robotic systems and defining humanoid robots using URDF. These concepts form the foundation for creating intelligent, humanoid robotic systems that can perceive, reason, and act in their environment.