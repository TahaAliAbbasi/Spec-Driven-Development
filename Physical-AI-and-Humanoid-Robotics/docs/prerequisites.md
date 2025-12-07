---
title: Prerequisites
sidebar_position: 0
---

# Prerequisites and System Requirements

This document outlines the minimum system requirements and prerequisites for effectively engaging with the "Physical AI & Humanoid Robotics" book project. Ensuring your environment meets these specifications will provide the best learning and development experience.

## 1. Operating System

*   **Primary**: Ubuntu 20.04 (Focal Fossa) or newer (e.g., Ubuntu 22.04 LTS Jammy Jellyfish).
    *   Recommended for ROS 2 and Gazebo development due to native support.
*   **Secondary**: Windows with WSL2 (Windows Subsystem for Linux 2).
    *   Allows running Ubuntu and ROS 2 tools on Windows. Performance may vary.

## 2. Hardware Requirements

### Minimum (for Docusaurus, basic ROS 2, and light Gazebo simulations)

*   **CPU**: Quad-core processor (e.g., Intel i5 / AMD Ryzen 5 or equivalent)
*   **RAM**: 8 GB
*   **Storage**: 50 GB free SSD space
*   **GPU**: Integrated graphics (Intel UHD, AMD Radeon integrated) may suffice for very simple Gazebo simulations, but dedicated GPU is highly recommended.

### Recommended (for NVIDIA Isaac Sim, advanced ROS 2, and complex Gazebo/Unity simulations)

*   **CPU**: Hexa-core or Octa-core processor (e.g., Intel i7 / AMD Ryzen 7 or equivalent)
*   **RAM**: 16 GB or 32 GB
*   **Storage**: 100 GB+ free NVMe SSD space
*   **GPU**: NVIDIA GeForce RTX 20 Series, NVIDIA Quadro RTX, or newer (e.g., RTX 30/40 series).
    *   **CUDA**: CUDA 11.8+ compatible GPU is mandatory for NVIDIA Isaac Sim and Isaac ROS.

## 3. Software Requirements

### 3.1. Development Tools

*   **Git**: Version control system.
    *   Installation: `sudo apt install git` (Ubuntu)
*   **Node.js (LTS)**: Required for Docusaurus development.
    *   Recommended installation via NVM: `curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash`
*   **Python 3.9+**: For ROS 2 (`rclpy`), AI agent scripts, and various tools.
    *   Installation: `sudo apt install python3 python3-pip` (Ubuntu)
*   **Docker & Docker Compose**: (Highly Recommended) For consistent and isolated development environments.
    *   Follow official Docker installation guides for your OS.

### 3.2. Robotics Software

*   **ROS 2 (Robotic Operating System 2)**:
    *   **Version**: Humble Hawksbill or Iron Irwini (latest LTS releases).
    *   **Installation**: Follow the [Official ROS 2 Documentation](https://docs.ros.org/en/humble/Installation.html).
*   **Gazebo Simulation (Fortress/Garden)**:
    *   **Integration**: Typically installed as part of ROS 2 desktop full installation (`ros-<distro>-desktop`).
    *   **Installation**: Refer to [Gazebo Documentation](https://gazebosim.org/docs).
*   **NVIDIA Isaac Sim (2023.1.1+)**: For advanced, GPU-accelerated robotics simulation.
    *   **Installation**: Requires NVIDIA Omniverse Launcher. Refer to [NVIDIA Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/install_python.html).
*   **NVIDIA Isaac ROS**: Hardware-accelerated packages for ROS 2.
    *   **Integration**: Used with Isaac Sim and ROS 2. Refer to [NVIDIA Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/getting_started/getting_started.html).

### 3.3. AI/LLM Components

*   **OpenAI Whisper**: For speech-to-text functionality.
    *   Installation: `pip install -U openai-whisper`
*   **Large Language Models (LLMs)**: Integration will depend on specific LLMs chosen for cognitive planning (e.g., local models like Llama.cpp, or API-based like OpenAI GPT models/Anthropic Claude models).
    *   Specific setup instructions will be provided in relevant modules.

## Verification

After installation, verify each component:

*   **Git**: `git --version`
*   **Node.js**: `node -v`, `npm -v`
*   **Python**: `python3 --version`, `pip3 --version`
*   **Docker**: `docker --version`, `docker compose version`
*   **ROS 2**: `ros2 --version`, `ros2 topic list` (after sourcing setup)
*   **Gazebo**: `gazebo --version` or `gz --version`
*   **NVIDIA GPU/CUDA**: `nvidia-smi`, `nvcc --version`

Ensuring all these prerequisites are met will enable a smooth and productive learning experience throughout the book project.
