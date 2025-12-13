# Data Models for Physical AI & Humanoid Robotics Capstone

This document defines the structure and components of the key data entities used in the project. These are not database schemas but rather descriptions of the core data structures and software components.

## 1. Curriculum

This entity represents the full syllabus for the capstone course. It is defined through the directory structure and content of the Docusaurus site.

**Fields/Structure**:
-   **`docs/`**: Main directory for course content.
    -   **`intro.md`**: Course overview, goals, and prerequisites.
    -   **`01-ros-fundamentals/`**: Directory for the ROS 2 module.
        -   `_category_.json`: Defines the sidebar name for this module.
        -   `part1.md`, `part2.md`: Individual pages for topics within the module.
    -   **`02-digital-twin/`**: Directory for the Simulation module.
    -   **`03-nvidia-isaac/`**: Directory for the NVIDIA Isaac module.
    -   **`04-vla-integration/`**: Directory for the Conversational AI module.
    -   **`capstone-project.md`**: Detailed instructions for the final project.
-   **`docusaurus.config.ts`**: Main configuration, including project name and theme.
-   **`sidebars.ts`**: Defines the navigation structure and order of all documents in the `docs` directory.

**Validation Rules**:
-   The sidebar structure defined in `sidebars.ts` must match the directory structure under `docs/`.
-   All modules listed in the user prompt must be represented as directories.

## 2. Humanoid Robot Model

This entity is the digital description of the humanoid robot, primarily defined in a URDF (Unified Robot Description Format) file.

**File Format**: URDF (.urdf) or XACRO (.urdf.xacro) for more modular definitions.

**Structure of the URDF**:
-   **`<robot name="...">`**: The root element.
-   **`<link name="...">`**: Defines a physical part of the robot (e.g., `torso`, `left_femur`, `right_hand`).
    -   **`<visual>`**: Defines the 3D mesh for visualization (e.g., a `.dae` or `.stl` file).
    -   **`<collision>`**: Defines the geometry for physics simulation.
    -   **`<inertial>`**: Defines the mass and inertia properties.
-   **`<joint name="..." type="...">`**: Defines the connection between two links.
    -   **`type`**: Can be `revolute` (for rotating joints), `prismatic` (for sliding joints), `fixed`, etc.
    -   **`<parent link="..." />`** and **`<child link="..." />`**: Defines the hierarchy.
    -   **`<axis xyz="..." />`**: The axis of rotation or movement.
    -   **`<limit lower="..." upper="..." effort="..." velocity="..." />`**: Defines the joint limits.
-   **`<sensor>`**: Defines sensors like cameras and IMUs and their attachment points.
-   **`<transmission>`**: Defines the relationship between a joint and an actuator.

**Validation Rules**:
-   The URDF must be well-formed XML.
-   The URDF must successfully parse in ROS 2 and Gazebo.
-   All mesh and material files referenced in the URDF must exist at the specified paths.

## 3. Simulation Environment

This entity defines the virtual world in which the robot is simulated. It is typically defined in an SDF (Simulation Description Format) file.

**File Format**: SDF (`.sdf`) or Gazebo World file (`.world`).

**Structure of the World file**:
-   **`<sdf version="...">`**: Root element.
-   **`<world name="...">`**: Defines the world.
    -   **`<physics type="ode">`**: Configures the physics engine (e.g., ODE, Bullet).
    -   **`<scene>`**: Configures visual properties like ambient light and shadows.
    -   **`<light type="..." name="...">`**: Defines light sources.
    -   **`<model name="...">`**: Includes a model in the world (e.g., the robot itself, tables, balls, walls).
        -   The robot model is included from its URDF file.
    -   **`<plugin name="..." filename="...">`**: Loads Gazebo plugins, for example, to interface with ROS 2.

## 4. AI Brain

This is a conceptual entity representing the collection of ROS 2 nodes and models that constitute the robot's intelligence.

**Components**:
-   **Perception Pipeline (NVIDIA Isaac ROS)**:
    -   Nodes for processing data from cameras (`image_proc`, `vslam`).
    -   Outputs: Odometry, recognized objects, 3D map of the environment.
-   **Navigation Stack (Nav2)**:
    -   Nodes for path planning, obstacle avoidance, and motion control.
    -   Inputs: Map, odometry, goal pose.
    -   Outputs: Velocity commands (`/cmd_vel`).
-   **Conversational AI (VLA)**:
    -   Node for speech-to-text (e.g., using OpenAI Whisper).
    -   Node for cognitive planning (LLM that translates natural language to a sequence of tasks).
    -   Node for action execution that translates tasks into ROS 2 service calls or action goals.

## 5. Hardware Kit

This entity represents the physical bill of materials for building the real-world robot.

**Structure**:
-   **Brain**: NVIDIA Jetson Orin Nano (8GB) / Orin NX (16GB).
-   **Vision**: Intel RealSense D435i / D455.
-   **Balance**: IMU (e.g., BNO055).
-   **Audio**: USB Microphone/Speaker (e.g., ReSpeaker Array).
-   **Chassis/Actuators**: The physical robot kit (e.g., Robotis OP3, Unitree G1).
-   **Power**: Batteries, power distribution board.
-   **Miscellaneous**: SD Card, jumper wires, USB cables.
