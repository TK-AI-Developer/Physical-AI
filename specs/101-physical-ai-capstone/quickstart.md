# Quickstart Guide: Physical AI & Humanoid Robotics Capstone

This guide provides the steps to set up the development environment for the capstone project.

## 1. Hardware Requirements (Workstation)

-   **GPU**: NVIDIA RTX 4070 Ti or higher (RTX 3090/4090 recommended for best performance).
-   **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9 (7000 series+).
-   **RAM**: 64 GB DDR5 (32 GB is the absolute minimum).
-   **Storage**: 1TB NVMe SSD (at least 200GB free space).

## 2. Operating System Installation

1.  **Install Ubuntu**: Download and install **Ubuntu 22.04 LTS** (Jammy Jellyfish).
2.  **Install NVIDIA Drivers**: After installation, install the recommended proprietary NVIDIA drivers for your graphics card. You can do this through the "Additional Drivers" application in Ubuntu.
    ```bash
    sudo ubuntu-drivers autoinstall
    sudo reboot
    ```
3.  **Verify Driver Installation**:
    ```bash
    nvidia-smi
    ```
    This command should show your GPU details and the driver version.

## 3. Core Software Installation

### Install ROS 2 Humble

Follow the official ROS 2 Humble installation guide for Ubuntu.

1.  **Set Locale**:
    ```bash
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 APT Repository**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 Desktop**:
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    ```
4.  **Source ROS 2**:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

### Install Gazebo

Gazebo is included with the `ros-humble-desktop` installation. Verify it by running:
```bash
gazebo --version
```

### Install Colcon

Colcon is the build tool for ROS 2.
```bash
sudo apt install python3-colcon-common-extensions
```

## 4. NVIDIA Isaac Sim Installation

1.  **Install NVIDIA Omniverse Launcher**: Download and install the Omniverse Launcher from the [NVIDIA Omniverse website](https://www.nvidia.com/en-us/omniverse/).
2.  **Install Isaac Sim**:
    -   Open the Omniverse Launcher.
    -   Go to the "Exchange" tab and search for "Isaac Sim".
    -   Select the latest version compatible with ROS 2 Humble (e.g., `2023.1.1`) and click "Install".
3.  **Run Isaac Sim**: Launch Isaac Sim from the "Library" tab in the Omniverse Launcher.

## 5. Project Workspace Setup

1.  **Create a Workspace**:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    ```
2.  **Clone the Project Repository** (This step is a placeholder for when the repo is available):
    ```bash
    cd ~/ros2_ws/src
    git clone <URL_TO_YOUR_PROJECT_REPO>
    ```
3.  **Install Dependencies and Build**:
    ```bash
    cd ~/ros2_ws
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build
    ```
4.  **Source the Workspace**:
    ```bash
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

## 6. "Hello World" - Running a Simple Simulation

1.  **Launch a Demo Simulation**:
    ```bash
    # This is an example command using a pre-packaged ROS 2 demo
    ros2 launch gazebo_ros turtlebot3_world.launch.py
    ```
2.  **Teleoperate the Robot**:
    Open a new terminal and run the teleoperation node:
    ```bash
s
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```
    You should now be able to drive the simulated robot using your keyboard. This confirms that your ROS 2 and Gazebo installation is working correctly.
