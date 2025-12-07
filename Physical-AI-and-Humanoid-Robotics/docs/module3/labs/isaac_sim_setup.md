---
title: NVIDIA Isaac Sim Setup Guide
sidebar_position: 1
---

# NVIDIA Isaac Sim Setup Guide

This guide provides detailed instructions for setting up NVIDIA Isaac Sim, a powerful robotics simulation platform. A properly configured Isaac Sim environment is essential for leveraging photorealistic simulation, synthetic data generation, and advanced AI-powered robotics development.

## 1. Prerequisites

Before proceeding, ensure you have:

*   **NVIDIA GPU**: A modern NVIDIA RTX GPU is highly recommended (e.g., RTX 20 series, 30 series, 40 series, or Quadro) for optimal performance and photorealistic rendering capabilities.
*   **Operating System**: Ubuntu 20.04 LTS or 22.04 LTS is typically recommended. Windows Subsystem for Linux (WSL2) with GPU passthrough can also be used, but native Linux is preferred for full compatibility.
*   **Docker and NVIDIA Container Toolkit**: Isaac Sim is often deployed via Docker containers. Ensure you have Docker and the NVIDIA Container Toolkit installed and configured correctly. Refer to the official [NVIDIA Container Toolkit documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).
*   **Internet Connection**: Required for downloading Isaac Sim and its dependencies.
*   **Omniverse Account**: An NVIDIA Omniverse account is required to download and access Isaac Sim.

## 2. Installing NVIDIA Omniverse Launcher

1.  **Download Omniverse Launcher**: Go to the [NVIDIA Omniverse website](https://developer.nvidia.com/omniverse) and download the Omniverse Launcher for your operating system.
2.  **Install Omniverse Launcher**: Follow the installation instructions for the Launcher.
3.  **Sign In**: Launch the Omniverse Launcher and sign in with your NVIDIA Omniverse account.

## 3. Installing Isaac Sim

1.  **Navigate to Exchange**: In the Omniverse Launcher, go to the "Exchange" tab.
2.  **Search for Isaac Sim**: Search for "Isaac Sim" and select it.
3.  **Install**: Click the "Install" button. The Launcher will download and install the latest version of Isaac Sim. This process might take some time as it downloads a large Docker image or local files.

## 4. Launching Isaac Sim

### Via Omniverse Launcher (GUI Mode)

1.  **Go to Library**: In the Omniverse Launcher, navigate to the "Library" tab.
2.  **Launch Isaac Sim**: Under "Apps", find Isaac Sim and click the "Launch" button. This will start Isaac Sim with its graphical user interface.

### Via Docker (Headless Mode for scripting/automation)

For automation, CI/CD, or server deployments, you might run Isaac Sim in headless mode using Docker. The exact command depends on your Isaac Sim version, but it generally follows this pattern:

```bash
docker run --name isaac-sim --privileged --gpus all -e "ACCEPT_EULA=Y" --network=host \
    -v ~/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/isaac-sim/data:/root/.local/share/ov/pkg/isaac_sim-2023.1.1/data:rw \
    -v ~/isaac-sim/documents:/root/Documents:rw \
    nvcr.io/nvidia/isaac-sim:2023.1.1
```

*   **`--gpus all`**: Grants access to all GPUs. Adjust if you have multiple GPUs and want to specify one.
*   **`--network=host`**: Allows the container to use the host's network stack, simplifying ROS 2 communication.
*   **`-e "ACCEPT_EULA=Y"`**: Automatically accepts the NVIDIA End User License Agreement.
*   **`-v ...`**: Mounts various cache, logs, and data directories from your host to the container for persistence and access.
*   **`nvcr.io/nvidia/isaac-sim:2023.1.1`**: Replace with the specific Isaac Sim Docker image tag you installed.

## 5. Verifying Installation and Running a Sample Scene

Once Isaac Sim is launched, you can verify the installation by:

1.  **Opening a Sample Scene**: In the Isaac Sim GUI, go to `File > Open` and navigate to `Isaac Examples > Hello World > hello_world.usd`. This will load a basic scene.
2.  **Playing the Simulation**: Click the "Play" button in the timeline controls to start the simulation. You should see objects interacting according to physics.
3.  **Running Python Examples**: Isaac Sim includes a `script_editor` (accessible via `Window > Script Editor`) where you can run Python scripts. Try running a simple script to spawn an object or control a robot (e.g., from `omni.isaac.examples`).

## 6. Integrating with ROS 2

Isaac Sim provides the `omni.isaac.ros2_bridge` extension for ROS 2 communication. This extension is typically enabled by default.

1.  **Enable ROS 2 Bridge**: In Isaac Sim, go to `Window > Extensions` and search for "ROS2 Bridge". Ensure it is enabled.
2.  **Launch ROS 2 Nodes**: In a separate terminal (outside the Docker container if running Isaac Sim in Docker, or on your host machine), source your ROS 2 environment and launch any ROS 2 nodes that will communicate with Isaac Sim (e.g., a teleop node, a navigation stack).
3.  **Add ROS 2 Components in Isaac Sim**: In your USD scene, you can add ROS 2 specific components to robots and sensors through the property panel. For example, to a camera, add a `ROS2 Camera` component to publish images to a ROS 2 topic.

By following this guide, you will have a functional NVIDIA Isaac Sim environment, ready for advanced robotics simulation and AI development.