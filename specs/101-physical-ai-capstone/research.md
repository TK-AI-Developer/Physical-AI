# Research for Physical AI & Humanoid Robotics Capstone

This document contains the research findings for the key technical decisions and areas needing clarification in the implementation plan.

## 1. C++ Standard for ROS 2 Nodes

**Decision**: The project will use **C++17**.

**Rationale**: 
- C++17 is the recommended and officially targeted C++ standard for both ROS 2 Humble and the upcoming ROS 2 Jazzy.
- It provides access to modern C++ features (like `std::optional`, `std::filesystem`, and structured bindings) that improve code clarity and safety, which is beneficial in a complex robotics project.
- While C++14 is the minimum, C++17 is widely adopted and well-supported by the ROS 2 community and tooling.

**Alternatives Considered**:
- **C++14**: The minimum required version. Rejected because it lacks many of the quality-of-life and safety features of C++17.
- **C++20**: Offers even more advanced features, but support in the ROS 2 ecosystem and compiler availability on the target platforms (especially embedded ones like Jetson) may not be as mature. Sticking to C++17 ensures better compatibility.

## 2. ROS 2 Distribution Selection

**Decision**: The project will standardize on **ROS 2 Humble Hawksbill**.

**Rationale**:
- **Long-Term Support (LTS)**: Humble is an LTS release with support until May 2027. For a capstone project that will be taught over multiple semesters, stability and long-term support are critical.
- **Maturity and Compatibility**: As of the project start, Humble is a very mature and stable distribution with excellent community support and wide compatibility with essential tools like NVIDIA Isaac Sim and Gazebo.
- **EOL of Iron**: ROS 2 Iron Irwini is a non-LTS release with an End-of-Life date in late 2024, making it unsuitable for a project starting in 2025.

**Alternatives Considered**:
- **ROS 2 Iron Irwini**: Rejected because it will be unsupported and EOL by the time the project begins.
- **ROS 2 Jazzy Jalisco (LTS)**: Released in May 2024, Jazzy is a viable LTS alternative. However, Humble is the more conservative and currently better-supported choice, as it may take time for all third-party packages and tools to fully support a brand new distribution. The project could be updated to Jazzy in future iterations.

## 3. NVIDIA Isaac Sim Version

**Decision**: The project will use the **latest available version of NVIDIA Isaac Sim that provides official support for ROS 2 Humble**.

**Rationale**:
- NVIDIA Isaac Sim versions are closely tied to the ROS 2 distributions they support. Using a version officially designated for Humble ensures that the ROS 2 bridge and other integrations will function correctly.
- The latest version will contain the most recent features, bug fixes, and performance improvements.

**Action Item**: The specific version number (e.g., Isaac Sim 2023.1.1) will be documented in the `quickstart.md` during Phase 1 design, after verifying the latest stable release on the NVIDIA developer website.

## 4. Sensor Selection and Calibration

**Decision**:
- **Selection**: The specified sensors (Intel RealSense D435i/D455 for vision, BNO055 or similar for IMU) are appropriate due to their strong ROS 2 support. The key is to use sensors with well-maintained, open-source ROS 2 drivers.
- **Calibration**: A standard, multi-step calibration process will be implemented.

**Rationale**:
- **Best Practices**: The chosen approach aligns with industry best practices for robust robotics perception. Accurate calibration is non-negotiable for successful sensor fusion and localization.
- **Standard Tooling**: Leveraging standard ROS packages like `camera_calibration` reduces development overhead and relies on community-vetted tools.
- **URDF as Source of Truth**: Storing extrinsic calibration (sensor poses) in the robot's URDF is the standard ROS methodology, allowing all components to share a single source of truth for the robot's geometry.

**Calibration Workflow**:
1.  **IMU**: Perform a static calibration to determine gyroscope and accelerometer biases.
2.  **Camera (Intrinsic)**: Use the ROS `camera_calibration` package with a checkerboard pattern to find the camera's internal parameters.
3.  **Camera/LiDAR (Extrinsic)**: Mount sensors securely on the robot. Accurately measure their position and orientation relative to the `base_link` and encode this into the robot's URDF file. Use calibration tools if higher precision is needed.
4.  **Verification**: Use Rviz to visualize all sensor data streams and confirm they are correctly transformed and aligned in the robot's coordinate frames.

## 5. Module & Workflow Prioritization

**Decision**: The project will adopt a **simulation-first** development workflow.

**Rationale**:
- **Safety and Cost**: Developing and testing on physical hardware from day one is risky and expensive. Simulation allows for rapid iteration and the testing of potentially dangerous maneuvers without risk of damaging the robot.
- **Speed of Development**: Bugs in algorithms (e.g., navigation, manipulation) can be identified and fixed much faster in a simulated environment where the state can be easily controlled and reset.
- **Sim-to-Real**: Modern simulators like Gazebo and Isaac Sim are designed for sim-to-real transfer. By developing in a high-fidelity simulation, the resulting code can be deployed to the physical robot with minimal changes, primarily for final tuning and validation.

**Workflow**:
1.  **Unit & Integration Testing**: Develop algorithms and ROS 2 nodes. Test them with simulated data (rostopic echo/pub) and unit tests.
2.  **Simulation Testing**: Deploy nodes to a simulated robot in Gazebo/Isaac Sim to test behavior in a 3D world.
3.  **Hardware Deployment**: Once the behavior is validated in simulation, deploy the code to the physical Jetson Orin and test on the real robot.

## 6. Workstation vs. Cloud Simulation

**Decision**: A **hybrid approach** will be recommended. The primary development environment will be a local workstation, with cloud instances offered as a managed alternative.

**Rationale**:
- **Local for Interactivity**: Robotics development requires low-latency, real-time feedback, especially when debugging control loops or interacting with a simulation. A powerful local workstation (like the specified RTX 4080 build) provides the best experience for this.
- **Cloud for Scalability & Accessibility**: Not all students may have access to the required high-end hardware. Providing pre-configured AWS g5 instances lowers the barrier to entry. The cloud also excels at large-scale, non-interactive tasks like training reinforcement learning models or generating massive synthetic datasets.
- **Latency Trap**: The plan must emphasize that while simulation can be done in the cloud, real-time control of the physical robot MUST be done from a local machine (the Jetson) due to network latency. The workflow will be: **train in the cloud, deploy and run on the edge**.

**Alternatives Considered**:
- **Local Only**: Excludes students without powerful hardware.
- **Cloud Only**: Unsuitable for the interactive, low-latency development and debugging phases.

## 7. Physical Humanoid vs. Proxy Robot

**Decision**: The project will recommend the **Robotis OP3** as the primary educational platform, with the **Unitree G1** as a premium alternative and the **Unitree Go2** (quadruped) as a suitable proxy for non-humanoid-specific learning goals.

**Rationale**:
- **Robotis OP3**: Offers the best balance of features, cost, and educational value for a humanoid platform. Its open-source nature and strong ROS support are ideal for a capstone project where students need to dig into the software and hardware.
- **Unitree G1**: A more advanced, but significantly more expensive, option. It's a good "premium" choice for well-funded labs that want to work with state-of-the-art hardware.
- **Unitree Go2 (Proxy)**: While not a humanoid, its excellent ROS 2 support and advanced sensor suite make it a valuable tool for teaching the perception, navigation, and AI pipeline components of the course. It can be used for most modules except those specifically focused on bipedal locomotion and humanoid manipulation.
- **Hiwonder TonyPi Pro**: Considered too entry-level for the advanced topics covered in this capstone, particularly the NVIDIA Isaac integration. It serves a different educational niche (Raspberry Pi-based AI learning).

**Tradeoffs**: The primary tradeoff is cost vs. capability. The OP3 provides a complete humanoid experience at a manageable price point for an educational lab, while the G1 offers higher performance for a much higher price.
