---
id: 101
title: Specify Physical AI Capstone Project
stage: spec
date: 2025-12-10
surface: agent
model: gemini
feature: 101-physical-ai-capstone
branch: 101-physical-ai-capstone
user: {{USER}}
command: /sp.specify
labels: [spec, capstone, robotics]
links:
  spec: specs/101-physical-ai-capstone/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/101-physical-ai-capstone/spec.md
 - specs/101-physical-ai-capstone/checklists/requirements.md
tests:
[]
---

## Prompt

# /sp.spec project: name: Physical AI & Humanoid Robotics description: > This capstone project focuses on bridging the gap between digital AI models and physical humanoid robots. Students will design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, Unity, and NVIDIA Isaac. focus_theme: AI Systems in the Physical World, Embodied Intelligence goal: > Enable students to apply AI knowledge to control humanoid robots in simulated and real-world environments. quarters: capstone_quarter: overview: > Introduces Physical AI—AI systems that operate in reality and understand physical laws. Students learn to design, simulate, and deploy humanoid robots using ROS 2, Gazebo, Unity, and NVIDIA Isaac. modules: - name: Robotic Nervous System (ROS 2) focus: Middleware for robot control topics: - ROS 2 Nodes, Topics, and Services - Bridging Python agents to ROS controllers (rclpy) - URDF for humanoids - name: Digital Twin (Gazebo & Unity) focus: Physics simulation and environment building topics: - Physics simulation: gravity, collisions - Sensor simulation: LiDAR, Depth Cameras, IMUs - High-fidelity rendering and human-robot interaction in Unity - name: AI-Robot Brain (NVIDIA Isaac) focus: Advanced perception and training topics: - Isaac Sim for photorealistic simulation & synthetic data generation - Isaac ROS: VSLAM and navigation - Nav2: Bipedal path planning - name: Vision-Language-Action (VLA) focus: LLMs and robotics convergence topics: - Voice-to-Action using OpenAI Whisper - Cognitive planning: natural language → ROS 2 actions - Capstone: Autonomous Humanoid executing tasks learning_outcomes: - Understand principles of Physical AI and embodied intelligence - Master ROS 2 for robot control - Simulate humanoid robots in Gazebo and Unity - Develop AI pipelines using NVIDIA Isaac - Integrate GPT models for conversational robotics - Design humanoid robots for natural human interactions weekly_breakdown: - weeks: 1-2 topic: Introduction to Physical AI content: > Foundations of embodied intelligence, humanoid robotics overview, sensor systems: LIDAR, cameras, IMUs, force/torque sensors - weeks: 3-5 topic: ROS 2 Fundamentals content: > ROS 2 architecture, nodes, topics, services, actions, Python packages, launch files - weeks: 6-7 topic: Robot Simulation with Gazebo content: > Gazebo setup, URDF/SDF robot description, physics and sensor simulation, Unity visualization - weeks: 8-10 topic: NVIDIA Isaac Platform content: > Isaac SDK, perception/manipulation, reinforcement learning, sim-to-real transfer - weeks: 11-12 topic: Humanoid Robot Development content: > Humanoid kinematics/dynamics, bipedal locomotion, manipulation, human-robot interaction - week: 13 topic: Conversational Robotics content: > GPT model integration, speech recognition, multimodal interaction (speech, gesture, vision) assessments: - ROS 2 package development - Gazebo simulation implementation - Isaac-based perception pipeline - Capstone: Simulated humanoid robot with conversational AI hardware_requirements: digital_twin_workstation: required: true specs: GPU: NVIDIA RTX 4070 Ti or higher (RTX 3090/4090 recommended) CPU: Intel i7 13th Gen+ or AMD Ryzen 9 RAM: 64 GB DDR5 (32 GB minimum) OS: Ubuntu 22.04 LTS edge_kit: required: true components: brain: NVIDIA Jetson Orin Nano (8GB) / Orin NX (16GB) vision: Intel RealSense D435i / D455 balance: IMU (BNO055) audio: USB Microphone/Speaker (ReSpeaker) robot_lab: options: proxy_approach: robot: Unitree Go2 Edu pros: Durable, ROS 2 support, affordable cons: Not humanoid miniature_humanoid: robot: Hiwonder TonyPi Pro / Unitree G1 / Robotis OP3 pros: Small, table-top cons: Some kits cannot run Isaac ROS premium_lab: robot: Unitree G1 Humanoid pros: Full biped, SDK open for custom ROS 2 controllers cloud_alternatives: high_opex: description: Cloud-native lab using AWS/Azure instances: AWS g5.2xlarge / g6e.xlarge sim: NVIDIA Isaac Sim Omniverse Cloud notes: Latency requires offline model flashing to Jetson kits budget_options: economy_jetson_student_kit: total_cost: ~$700 components: brain: NVIDIA Jetson Orin Nano Super Dev Kit eyes: Intel RealSense D435i ears: ReSpeaker USB Mic Array WiFi: Included misc: SD Card (128GB), jumper wires notes: latency_trap: > Cloud simulation is effective, but real robot control must be local. Students train in cloud, then flash models to Jetson kit.

## Response snapshot

I have created the feature specification for the "Physical AI & Humanoid Robotics Capstone" project.

- **Branch**: `101-physical-ai-capstone`
- **Specification**: `specs/101-physical-ai-capstone/spec.md`
- **Checklist**: `specs/101-physical-ai-capstone/checklists/requirements.md`

The specification is ready for the next phase.

## Outcome

- ✅ Impact: {{OUTCOME_IMPACT}}
- 🧪 Tests: {{TESTS_SUMMARY}}
- 📁 Files: {{FILES_SUMMARY}}
- 🔁 Next prompts: {{NEXT_PROMPTS}}
- 🧠 Reflection: {{REFLECTION_NOTE}}

## Evaluation notes (flywheel)

- Failure modes observed: {{FAILURE_MODES}}
- Graders run and results (PASS/FAIL): {{GRADER_RESULTS}}
- Prompt variant (if applicable): {{PROMPT_VARIANT_ID}}
- Next experiment (smallest change to try): {{NEXT_EXPERIMENT}}
