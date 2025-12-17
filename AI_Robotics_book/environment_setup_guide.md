# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Module

This quickstart guide provides a concise, step-by-step introduction to setting up the core development environment and running a basic ROS 2 example. It covers both Ubuntu and Windows (via WSL2) setup, aiming to get you started within 4 hours.

## 1. Choose Your Environment

### Option A: Native Ubuntu 22.04 LTS (Recommended for best performance and compatibility)

If you have a dedicated machine or are comfortable with dual-booting, this is the most friction-free experience.

### Option B: Windows with WSL2 (Windows Subsystem for Linux 2)

This option allows Windows users to run a full Ubuntu environment seamlessly alongside Windows.

## 2. Hardware and Software Prerequisites

### Digital Twin Workstation (Required)
- **GPU**: NVIDIA RTX 4070 Ti (12GB VRAM) or higher. (Crucial for Isaac Sim)
- **CPU**: Intel Core i7 (13th Gen+) or AMD Ryzen 9.
- **RAM**: 64 GB DDR5 (32 GB absolute minimum).
- **Disk Space**: At least 200 GB free SSD space.

### Software (Common to both options)
- **Ubuntu 22.04 LTS**: The base operating system for your development environment.
- **ROS 2 Humble/Iron**: Robotic Operating System 2 distribution.
- **NVIDIA Drivers**: Latest proprietary drivers for your GPU.
- **NVIDIA Container Toolkit (Docker)**: For running Isaac Sim and Isaac ROS containers.
- **NVIDIA Isaac Sim**: For photorealistic simulation and synthetic data generation.

## 3. Environment Setup

### Option A: Native Ubuntu 22.04 LTS Setup

1.  **Install Ubuntu 22.04 LTS**:
    *   Perform a clean installation of Ubuntu 22.04 LTS.
    *   Ensure all system updates are applied (`sudo apt update && sudo apt upgrade`).

2.  **Install NVIDIA Drivers**:
    *   Open "Software & Updates" -> "Additional Drivers".
    *   Select the latest NVIDIA proprietary driver and install it. Reboot.
    *   Verify installation: `nvidia-smi`

3.  **Install ROS 2 Humble/Iron**:
    *   Follow the official ROS 2 installation guide for Ubuntu 22.04: [ROS 2 Humble Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (or Iron, if specified).
    *   Remember to source your ROS 2 setup file in your `~/.bashrc`.

4.  **Install NVIDIA Container Toolkit**:
    *   Follow the official installation guide: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

5.  **Install NVIDIA Isaac Sim**:
    *   Download and install Omniverse Launcher from NVIDIA.
    *   Use the Omniverse Launcher to install Isaac Sim. Ensure you select the correct version compatible with your setup.

### Option B: Windows with WSL2 Setup

1.  **Enable WSL2 and Install Ubuntu 22.04 LTS**:
    *   Open PowerShell as Administrator and run:
        ```powershell
        wsl --install
        wsl --install -d Ubuntu-22.04
        ```
    *   Follow prompts to set up your Ubuntu username and password.
    *   Set WSL2 as default: `wsl --set-default-version 2`

2.  **Install Windows NVIDIA GPU Driver**:
    *   Ensure your Windows NVIDIA GPU driver is up-to-date. WSL2 will leverage this.

3.  **Install ROS 2 Humble/Iron (inside WSL2 Ubuntu)**:
    *   Open your WSL2 Ubuntu terminal.
    *   Follow the official ROS 2 installation guide for Ubuntu 22.04: [ROS 2 Humble Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
    *   Remember to source your ROS 2 setup file in your `~/.bashrc`.

4.  **Install NVIDIA Container Toolkit (inside WSL2 Ubuntu)**:
    *   Follow the official installation guide for Ubuntu: [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

5.  **Install NVIDIA Isaac Sim (on Windows)**:
    *   Download and install Omniverse Launcher on your Windows host.
    *   Use the Omniverse Launcher to install Isaac Sim.
    *   **Crucial for WSL2**: Configure Isaac Sim to connect to the ROS 2 running inside your WSL2 instance. This usually involves network configuration and potentially a small ROS bridge. Detailed instructions will be provided in Module 2.

## 4. Run Your First ROS 2 Example

1.  **Open a Terminal**:
    *   **Ubuntu Native**: Open a standard terminal.
    *   **WSL2**: Open your WSL2 Ubuntu terminal.

2.  **Source ROS 2**:
    ```bash
    source /opt/ros/humble/setup.bash # or iron, depending on your install
    source ~/ros2_ws/install/setup.bash # if you have a workspace
    ```

3.  **Run the talker-listener example**:
    *   In one terminal: `ros2 run demo_nodes_py talker`
    *   In a second terminal (after sourcing ROS 2): `ros2 run demo_nodes_py listener`

    You should see messages being exchanged between the talker and listener nodes.

This completes your basic setup and verifies ROS 2 functionality. Proceed to Module 1 for in-depth learning!