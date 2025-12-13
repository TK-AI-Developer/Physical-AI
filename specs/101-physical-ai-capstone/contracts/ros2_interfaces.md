# ROS 2 Interfaces for Physical AI & Humanoid Robotics Capstone

This document defines the key ROS 2 topic, service, and action interfaces that form the primary "contracts" between the different nodes in the robotic system.

## 1. Topics

Topics are used for continuous streams of data.

---

### `/cmd_vel`

-   **Message Type**: `geometry_msgs/Twist`
-   **Direction**: Published by the Navigation stack (Nav2) or a teleoperation node. Subscribed to by the robot's base controller.
-   **Purpose**: Sends velocity commands to the mobile base of the robot.
-   **Structure** (`geometry_msgs/Twist`):
    ```
    # Linear velocities
    Vector3  linear
      float64 x
      float64 y
      float64 z
    # Angular velocities
    Vector3  angular
      float64 x
      float64 y
      float64 z
    ```

---

### `/joint_states`

-   **Message Type**: `sensor_msgs/JointState`
-   **Direction**: Published by the robot's hardware interface/controller. Subscribed to by RViz, the robot state publisher, and any node needing to know the robot's current configuration.
-   **Purpose**: Broadcasts the current state (position, velocity, effort) of all the robot's joints.
-   **Structure** (`sensor_msgs/JointState`):
    ```
    Header header
    string[] name         # Array of joint names
    float64[] position    # Array of joint positions (radians or meters)
    float64[] velocity    # Array of joint velocities
    float64[] effort      # Array of joint efforts
    ```

---

### `/odom`

-   **Message Type**: `nav_msgs/Odometry`
-   **Direction**: Published by a sensor fusion node (like `robot_localization`) or a wheel odometry node. Subscribed to by the navigation stack.
-   **Purpose**: Provides an estimate of the robot's position and velocity in its odometry frame.
-   **Structure** (`nav_msgs/Odometry`):
    ```
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    ```

## 2. Services

Services are used for request/response interactions.

---

### `/execute_named_task`

-   **Service Type**: `custom_interfaces/srv/ExecuteNamedTask` (custom definition)
-   **Direction**: Called by the conversational AI's action execution node. Implemented by a robot skill or behavior node.
-   **Purpose**: Commands the robot to perform a discrete, pre-defined task like "wave_hand" or "return_to_home".
-   **Structure** (`ExecuteNamedTask.srv`):
    ```
    # Request
    string task_name
    ---
    # Response
    bool success
    string message
    ```

## 3. Actions

Actions are used for long-running, feedback-producing tasks that can be preempted.

---

### `/navigate_to_pose`

-   **Action Type**: `nav2_msgs/action/NavigateToPose` (from Nav2 stack)
-   **Direction**: Goal sent by a high-level planner (like the conversational AI node). Implemented by the Nav2 navigation server.
-   **Purpose**: Commands the robot to navigate to a specific pose (position and orientation) on the map.
-   **Structure** (`NavigateToPose.action`):
    ```
    # Goal
    geometry_msgs/PoseStamped pose
    ---
    # Result
    std_srvs/Empty result
    ---
    # Feedback
    geometry_msgs/PoseStamped current_pose
    builtin_interfaces/Duration navigation_time
    float32 distance_remaining
    ```
