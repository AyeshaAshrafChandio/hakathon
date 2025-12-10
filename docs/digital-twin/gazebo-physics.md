---
sidebar_position: 11
---

# Gazebo Physics Simulation

## Introduction

Gazebo is a powerful physics simulation environment that serves as a cornerstone for robotics development. It provides realistic physics simulation with accurate gravity, collision detection, and environmental modeling. This chapter explores how to configure and utilize Gazebo for simulating robot behavior in realistic environments.

## Core Physics Concepts in Gazebo

### Gravity and Environmental Forces

Gazebo simulates gravity by default with a standard Earth gravity of 9.8 m/s². This can be customized in the world file:

```xml
<sdf version='1.7'>
  <world name='default'>
    <physics type='ode'>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

### Collision Detection

Gazebo uses multiple physics engines for collision detection:
- **ODE (Open Dynamics Engine)**: Default engine, good for general purpose simulation
- **Bullet**: More stable for complex interactions
- **Simbody**: Better for biomechanics and complex joints
- **DART**: Advanced contact modeling

### Material Properties

Different materials can be defined with specific physical properties:

```xml
<material name='blue'>
  <ambient>0.2 0.2 0.8 1.0</ambient>
  <diffuse>0.2 0.2 0.8 1.0</diffuse>
  <specular>0.2 0.2 0.8 1.0</specular>
</material>
```

## Robot-Environment Interactions

### Joint Dynamics

Joints in Gazebo can be configured with various physical properties:

```xml
<joint name="hinge_joint" type="revolute">
  <parent>link1</parent>
  <child>link2</child>
  <axis>
    <xyz>0 0 1</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
      <effort>100</effort>
      <velocity>1</velocity>
    </limit>
    <dynamics>
      <damping>0.1</damping>
      <friction>0.2</friction>
    </dynamics>
  </axis>
</joint>
```

### Contact Sensors

Contact sensors can detect when objects collide:

```xml
<sensor name="contact_sensor" type="contact">
  <contact>
    <collision>link_collision</collision>
  </contact>
  <update_rate>30</update_rate>
</sensor>
```

## Setting Up a Physics Simulation Environment

### Creating a Basic World

A basic Gazebo world file includes the physics engine configuration and environment elements:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="basic_world">
    <!-- Include atmosphere -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A simple box obstacle -->
    <model name="box">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.1667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1667</iyy>
            <iyz>0</iyz>
            <izz>0.1667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Configuring Physics Parameters

Physics parameters can be tuned for different simulation requirements:

- **Max Step Size**: Smaller values provide more accurate simulation but require more computation
- **Real Time Factor**: Ratio of simulation time to real time (1.0 = real-time)
- **Update Rate**: How frequently physics calculations are updated

## Humanoid Robot Physics Considerations

### Balance and Stability

Humanoid robots require special attention to physics properties for stable simulation:

```xml
<!-- Example of a humanoid foot with proper friction -->
<collision name="left_foot_collision">
  <geometry>
    <box>
      <size>0.2 0.1 0.05</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

### Center of Mass

Proper center of mass configuration is critical for humanoid stability:

```xml
<inertial>
  <mass>10.0</mass>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <inertia>
    <ixx>0.5</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>0.5</iyy>
    <iyz>0</iyz>
    <izz>0.3</izz>
  </inertia>
</inertial>
```

## Quality of Life Tips

### Simulation Performance

- Use simplified collision geometries where possible (boxes instead of complex meshes)
- Adjust update rates based on required accuracy
- Use fixed joints instead of very stiff constraints

### Debugging Physics Issues

Common physics issues and solutions:
- **Objects falling through the ground**: Check collision geometries and physics parameters
- **Jittery motion**: Reduce step size or increase damping
- **Unstable joints**: Check joint limits and dynamics parameters

## Integration with ROS 2

Gazebo integrates seamlessly with ROS 2 through gazebo_ros packages:

```xml
<plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
  <robotNamespace>/robot</robotNamespace>
</plugin>
```

This allows ROS 2 nodes to control simulated robots using the same interfaces as real robots.

## Exercises

1. **Basic Physics Setup**: Create a simple world with gravity and collision detection
2. **Joint Configuration**: Configure different joint types with appropriate physics properties
3. **Humanoid Stability**: Set up a simple humanoid model and tune physics parameters for stability
4. **Sensor Integration**: Add contact sensors to detect collisions in the simulation

## Summary

This chapter covered the fundamentals of physics simulation in Gazebo, focusing on gravity, collision detection, and robot-environment interactions. Understanding these concepts is essential for creating realistic simulation environments that can be used for humanoid robotics development and testing.