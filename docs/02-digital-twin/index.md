---
id: module-2
title: "Module 2: The Digital Twin (Gazebo & Unity)"
sidebar_label: Digital Twin
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Overview
This module explores the creation and utilization of digital twins for robotic systems, focusing on Gazebo for physics-based simulation and Unity for high-fidelity visualization. Digital twins provide a safe, cost-effective, and flexible environment for developing, testing, and debugging robot behaviors before deployment to physical hardware. We will learn to construct virtual environments, simulate robot dynamics, and integrate sensor models.

## Key Concepts / Topics
- **Gazebo Simulation**: Building virtual worlds, defining physics properties, and simulating robot models within Gazebo.
- **Unified Robot Description Format (URDF)**: Describing robot kinematics, dynamics, and visual properties for simulation.
- **Sensor Simulation**: Implementing realistic models for LiDAR, depth cameras, IMUs, and other sensors.
- **ROS-Gazebo Integration**: Bridging Gazebo simulation data with ROS 2 topics, services, and actions.
- **Unity Robotics**: Utilizing Unity's advanced rendering capabilities for enhanced visualization and human-robot interaction.
- **Sim-to-Real Transfer**: Strategies for ensuring simulated behaviors translate effectively to real-world robots.

## Practical Examples / Exercises
We will develop a basic Gazebo world and integrate a simple robot URDF. We will also explore how to spawn robots in the simulation and visualize sensor data.

### Example: Basic Gazebo World (SDF snippet)
A minimal SDF file defining a ground plane and a light source.

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>1</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
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
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

### Example: Robot Spawner (ROS 2 Launch file snippet)
A Python launch file to spawn a URDF model in Gazebo.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('robot_description') # Assuming robot_description pkg
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')

    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'my_robot', '-file', urdf_file],
            output='screen'
        )
    ])
```

## Digital Twin Implementation Notes
Effective digital twin creation requires careful attention to detail in URDF/SDF modeling, accurate physics parameters, and calibration of simulated sensors. Regularly validating the simulation against real-world data is essential for achieving good sim-to-real transfer. Tools like RViz and Gazebo's GUI are invaluable for debugging and visualizing the virtual robot and environment.