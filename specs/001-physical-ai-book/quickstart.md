# Quickstart Guide: Physical AI & Humanoid Robotics Book Project

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-05 | **Spec**: specs/001-physical-ai-book/spec.md

## Overview

This guide provides a quick and concise walkthrough to set up your development environment and get started with the "Physical AI & Humanoid Robotics" book project. Follow these steps to begin contributing or running simulations.

## 1. Prerequisites

Before you begin, ensure you have the following installed:

*   **Git**: For cloning the repository.
*   **Node.js (LTS)**: Required for Docusaurus.
*   **Python 3.9+**: For ROS 2 (`rclpy`) and AI agent development.
*   **Docker & Docker Compose**: (Recommended) For consistent development environments.
*   **Ubuntu 20.04+ (or WSL2 on Windows)**: For ROS 2 and Gazebo.
*   **NVIDIA GPU (RTX 20 series or newer)**: Recommended for NVIDIA Isaac Sim and high-performance simulations.

## 2. Repository Setup

1.  **Clone the repository**:
    ```bash
    git clone https://github.com/your-org/physical-ai-book.git
    cd physical-ai-book
    ```
    *(Replace `your-org/physical-ai-book.git` with the actual repository URL)*

2.  **Install Docusaurus dependencies**:
    ```bash
    npm install
    ```

## 3. Docusaurus Local Development

To run the Docusaurus site locally:

1.  **Start the development server**:
    ```bash
    npm run start
    ```
    This will open the book in your browser at `http://localhost:3000`.

2.  **Build the static site (for deployment)**:
    ```bash
    npm run build
    ```
    The static files will be generated in the `build/` directory.

## 4. ROS 2 & Gazebo Environment Setup (Linux/WSL2)

For detailed ROS 2 and Gazebo installation, refer to the respective official documentation. Below is a condensed overview.

1.  **Install ROS 2 (Humble/Iron)**: Follow the official ROS 2 documentation for your Ubuntu version.
    ```bash
    # Example for Ubuntu 22.04 (Humble)
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    ```

2.  **Source ROS 2 setup**:
    ```bash
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

3.  **Install `rclpy` and other Python dependencies**:
    ```bash
    pip install rclpy
    # Install other Python dependencies as needed from requirements.txt or setup.py

### 4.4 Dockerized ROS 2 & Gazebo Environment (Recommended)

For a consistent and isolated development environment, it is highly recommended to use Docker.

1.  **Create a `Dockerfile` for your ROS 2 / Gazebo environment**:

    ```dockerfile
    # Dockerfile for ROS 2 Humble + Gazebo
    FROM osrf/ros:humble-desktop

    # Install common ROS 2 development tools and Gazebo dependencies
    RUN apt update && apt install -y \\
        build-essential \\
        cmake \\
        git \\
        python3-pip \\
        && rm -rf /var/lib/apt/lists/*

    # Install rclpy and other Python dependencies
    RUN pip install --no-cache-dir rclpy \\
        colcon-common-extensions

    # Set up ROS 2 entrypoint
    CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && bash"]
    ```

2.  **Create a `docker-compose.yml` for easy management**:

    ```yaml
    version: '3.8'
    services:
      ros_dev:
        build:
          context: .
          dockerfile: Dockerfile.ros
        container_name: ros_dev_container
        environment:
          - DISPLAY=${DISPLAY}
          - XDG_RUNTIME_DIR=/tmp/runtime-${USER}
          - ROS_DOMAIN_ID=0 # Or set a specific domain ID
        volumes:
          - /tmp/.X11-unix:/tmp/.X11-unix:rw # For GUI applications (Gazebo, RViz)
          - ./ros_workspace:/ros_workspace # Your ROS 2 workspace
          - /dev:/dev # Access to devices
        privileged: true # Required for some Gazebo/hardware interactions
        network_mode: "host" # Allows direct access to host network
        command: bash -c "source /opt/ros/humble/setup.bash && colcon build && bash"
    ```

    *   **Note**: You might need to adjust `DISPLAY` and `XDG_RUNTIME_DIR` based on your host OS configuration for GUI applications to work correctly.
    *   **`Dockerfile.ros`**: Save the Dockerfile above as `Dockerfile.ros` in your project root.
    *   **`ros_workspace`**: Create a directory named `ros_workspace` in your project root to serve as your ROS 2 workspace.

3.  **Build and run your Docker environment**:

    ```bash
    # Build the Docker image
    docker compose build

    # Start the development container
    docker compose up -d

    # Enter the container's shell
    docker exec -it ros_dev_container bash
    ```

This setup provides a portable and consistent environment for all ROS 2 and Gazebo development tasks.

    ```

## 5. NVIDIA Isaac Sim Setup (Optional, Recommended for Modules 3 & 4)

Refer to the official NVIDIA Isaac Sim documentation for installation. This typically involves downloading the Omniverse Launcher and installing Isaac Sim from there.

*   **Documentation**: [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)

## 6. AI Agent Interaction (Claude Code CLI)

To leverage AI-assisted content generation:

1.  **Ensure Claude Code CLI is installed and configured**.
2.  **Use SpecKit Plus commands**: Follow the project's Spec-Driven Development guidelines.
    *   `/sp.plan`: To generate/update planning artifacts.
    *   `/sp.tasks`: To generate implementation tasks.
    *   `/sp.phr`: To record Prompt History Records.

## 7. Contribution Guidelines

Refer to the project's `.github/CONTRIBUTING.md` (if available) or the "Contribution & Quality Workflow" section in `.specify/memory/constitution.md` for details on how to contribute.
