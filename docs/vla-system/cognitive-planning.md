---
sidebar_position: 27
---

# Cognitive Planning with LLMs

## Introduction

This chapter explores how Large Language Models (LLMs) perform cognitive planning by translating natural language instructions into detailed ROS 2 action plans. Cognitive planning represents the intelligence layer of the Vision-Language-Action (VLA) pipeline, bridging high-level human instructions with low-level robotic actions.

## Learning Objectives

By the end of this chapter, students will be able to:
- Explain how LLMs translate natural language to structured action plans
- Understand the process of generating ROS 2 action sequences from instructions
- Describe prompt engineering techniques for robotic applications
- Identify the challenges and solutions in natural language to action mapping

## Natural Language to Action Translation

### Understanding the Planning Process

Cognitive planning with LLMs involves transforming high-level natural language instructions into detailed, executable action sequences. This process requires the LLM to understand:

- **Intent Recognition**: Identifying the goal or desired outcome from the natural language
- **Task Decomposition**: Breaking complex instructions into smaller, manageable steps
- **World Modeling**: Understanding the environment and available actions
- **Action Sequencing**: Arranging actions in the correct order for successful execution

### Example Translation Process

Consider the natural language instruction: "Please go to the kitchen, find a red apple, and bring it to the dining table"

The LLM cognitive planning process would generate:
1. **Goal Analysis**: Deliver a red apple from the kitchen to the dining table
2. **Task Decomposition**: Navigate → Detect → Manipulate → Navigate
3. **Action Sequence**:
   - Navigate to kitchen
   - Detect red apple object
   - Grasp the apple
   - Navigate to dining table
   - Release the apple

### Context and Knowledge Integration

Effective cognitive planning requires the LLM to integrate:
- Knowledge about robot capabilities and limitations
- Understanding of the environment and object properties
- Awareness of physical constraints and safety considerations
- Ability to handle ambiguous or incomplete instructions

## ROS 2 Action Plan Generation

### Mapping to ROS 2 Actions

LLMs generate action plans by mapping natural language concepts to specific ROS 2 action interfaces. This involves:

- **Action Client Creation**: Instantiating appropriate ROS 2 action clients
- **Goal Definition**: Structuring action goals according to ROS 2 message formats
- **Parameter Mapping**: Converting natural language parameters to ROS 2 message fields
- **Execution Sequencing**: Organizing actions in the correct temporal order

### Example ROS 2 Action Plan

For the instruction "Move the robot 2 meters forward and turn left":

The generated ROS 2 action plan might include:
```python
# Action 1: Move forward
move_action_client = ActionClient('move_base', MoveBaseAction)
goal = MoveBaseGoal()
goal.target_pose.position.x = 2.0  # Move 2 meters forward
goal.target_pose.orientation.z = 0.0
move_action_client.send_goal(goal)

# Action 2: Turn left
rotate_action_client = ActionClient('rotate', RotateAction)
goal = RotateGoal()
goal.angle = 90.0  # Turn 90 degrees left
rotate_action_client.send_goal(goal)
```

### Handling Complex Action Sequences

For complex instructions, cognitive planning must handle:
- Conditional execution based on sensor feedback
- Error recovery and fallback strategies
- Dynamic replanning based on environmental changes
- Coordination between multiple robot subsystems

## Prompt Engineering for Robotics

### Effective Prompting Strategies

Successful cognitive planning with LLMs requires careful prompt engineering:

- **Clear Instruction Format**: Use consistent formats for instructions
- **Context Provision**: Provide relevant environmental and robot state information
- **Step-by-Step Reasoning**: Encourage the LLM to think through the plan step-by-step
- **Output Formatting**: Specify the expected format for action plans

### Example Prompt Template

```
You are a cognitive planning system for a mobile manipulator robot.
Given the robot state and environment, generate a sequence of actions to complete the user's request.

Robot capabilities: navigation, object detection, manipulation
Current location: entrance hall
Environment: kitchen (north), dining room (east), bedroom (west)

User request: {{user_instruction}}

Think step-by-step:
1. What is the goal?
2. What actions are needed?
3. What is the sequence of actions?

Generate ROS 2 action plan:
```

### Specialized Robotics Prompts

For specific robotic tasks, prompts can be specialized:
- **Navigation Tasks**: Include map information and obstacle data
- **Manipulation Tasks**: Include object properties and grasp planning
- **Perception Tasks**: Include sensor specifications and detection requirements
- **Multi-robot Tasks**: Include coordination and communication requirements

## Exercises

1. **Translation Exercise**: Convert the following natural language instruction to a high-level action sequence: "Go to the bookshelf, find the blue book, and place it on the desk."

2. **Prompt Design**: Design a prompt template for a robot that needs to sort objects by color into different bins.

3. **Error Handling**: Describe how cognitive planning should handle a situation where the LLM-generated plan fails during execution.

## Summary

This chapter covered the critical role of Large Language Models in cognitive planning for robotics. Understanding how LLMs translate natural language to structured action plans is essential for implementing intelligent robotic systems that can respond to human instructions effectively.