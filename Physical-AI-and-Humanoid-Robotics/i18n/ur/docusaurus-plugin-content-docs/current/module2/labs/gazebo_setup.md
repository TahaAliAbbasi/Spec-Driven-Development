---
title: Gazebo Environment Setup Guide
sidebar_position: 1
---

# Gazebo Environment Setup Guide

This guide provides detailed instructions for setting up your Gazebo simulation environment. A properly configured environment is crucial for realistic robot simulations and seamless integration with ROS 2.

## 1. Prerequisites

Before proceeding, ensure you have:

*   **Ubuntu Operating System**: Gazebo and ROS 2 are primarily developed for Ubuntu (e.g., Ubuntu 20.04 Focal Fossa for ROS 2 Foxy/Humble, Ubuntu 22.04 Jammy Jellyfish for ROS 2 Humble/Iron).
*   **ROS 2 Installation**: A working installation of ROS 2 (e.g., Humble or Iron) is required. Refer to the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html) for your specific Ubuntu version.
*   **Sufficient Hardware**: A modern CPU, at least 8GB RAM, and a dedicated GPU (NVIDIA recommended for optimal performance) are advisable for complex simulations.

## 2. Installing Gazebo (Standalone)

If you have already installed ROS 2 Desktop Full, Gazebo (usually `Gazebo Sim` or `Ignition Gazebo`) might already be installed. If not, or if you prefer a standalone installation, follow these steps:

### For Gazebo Garden (Recommended with ROS 2 Humble/Iron)

Gazebo Garden is the current generation and is often bundled with recent ROS 2 distributions.

1.  **Add Gazebo Repository (if not already added by ROS 2 installation)**:
    ```bash
sudo apt update
sudo apt install software-properties-common -y
sudo add-apt-repository "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main"
wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
    ```

2.  **Install Gazebo Garden**: (Adjust if you want a different version like Fortress)
    ```bash
sudo apt update
sudo apt install ignition-garden
    ```

### For Gazebo Classic (Gazebo 11 - with ROS 2 Foxy/Galactic)

If you are using an older ROS 2 distribution that relies on Gazebo Classic (version 11):

1.  **Add Gazebo Repository**:
    ```bash
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    ```

2.  **Install Gazebo 11**: (Ensure `ros-foxy-gazebo-ros-pkgs` or similar is installed if using with ROS 2)
    ```bash
sudo apt update
sudo apt install gazebo11
sudo apt install libgazebo11-dev
    ```

## 3. Verifying Gazebo Installation

After installation, you can launch Gazebo to ensure it's working correctly.

### For Gazebo Garden:

```bash
ign gazebo
```

### For Gazebo Classic:

```bash
gazebo
```

This should open the Gazebo GUI with an empty world. You can then insert models from the online database or create your own.

## 4. Integrating Gazebo with ROS 2

To allow ROS 2 nodes to interact with Gazebo, you need to use ROS 2 Gazebo plugins (often part of `gazebo_ros_pkgs`). These packages provide interfaces for publishing sensor data, subscribing to control commands, and loading URDF robot models.

1.  **Install ROS 2 Gazebo Bridge Packages**: (Example for Humble)
    ```bash
sudo apt install ros-humble-gazebo-ros-pkgs
    ```
    (Replace `humble` with your ROS 2 distribution, e.g., `foxy`, `galactic`, `iron`)

2.  **Include ROS 2 Gazebo Plugins in Robot Models**: When creating URDF/SDF models, you'll need to add `<plugin>` tags to specify which ROS 2 Gazebo plugins to load. For instance, `libgazebo_ros_diff_drive.so` for wheeled robots or `libgazebo_ros_joint_state_publisher.so` for joint states.

    **Example snippet in a URDF `.xacro` file:**

    ```xml
    <robot>
      <!-- ... robot definition ... -->
      <gazebo>
        <plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">
          <robotNamespace>/</robotNamespace>
          <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
        </plugin>
      </gazebo>
    </robot>
    ```

    This plugin allows you to use `ros2_control` to manage your robot's joints.

## 5. Basic Gazebo Commands

*   **Launch Gazebo with a specific world**: `gazebo --verbose path/to/your_world.world`
*   **Spawn a robot model**: `ros2 launch gazebo_ros spawn_entity.launch.py entity_name:=my_robot robot_name:=my_robot x_pos:=0 y_pos:=0 z_pos:=0 robot_urdf:=$(find my_robot_description)/urdf/my_robot.urdf`
*   **Record simulation data**: Gazebo allows logging simulation data for later analysis.

By following these steps, you will have a functional Gazebo environment integrated with ROS 2, ready for developing and testing your physical AI and humanoid robotics applications.