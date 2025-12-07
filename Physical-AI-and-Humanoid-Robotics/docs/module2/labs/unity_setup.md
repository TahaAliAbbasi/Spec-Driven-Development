---
title: Unity Environment Setup Guide
sidebar_position: 2
---

# Unity Environment Setup Guide

This guide provides comprehensive instructions for setting up your Unity development environment for robotics simulation and human-robot interaction (HRI) with ROS 2. Unity's powerful rendering and interactive capabilities offer a significant advantage for visualizing and controlling simulated robots.

## 1. Prerequisites

Before you begin, ensure you have:

*   **Modern Computer**: A system with a dedicated GPU (NVIDIA or AMD recommended), at least 16GB RAM, and a multi-core CPU for optimal Unity Editor performance and complex simulations.
*   **ROS 2 Installation**: A functional ROS 2 environment (e.g., Humble, Iron) is necessary for communication. Refer to the official [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html).
*   **Internet Connection**: Required for downloading Unity Hub, Editor, and various packages.

## 2. Installing Unity Hub and Unity Editor

1.  **Download Unity Hub**: Go to the official [Unity website](https://unity.com/download) and download Unity Hub. Unity Hub is a management tool for your Unity projects and Editor installations.
2.  **Install Unity Hub**: Follow the on-screen instructions to install Unity Hub.
3.  **Install Unity Editor**: Open Unity Hub and go to the "Installs" tab. Click "Install Editor" and choose a recommended LTS (Long Term Support) version (e.g., 2022 LTS or newer). Ensure you select the "Linux Build Support (IL2CPP)" and "Windows Build Support (IL2CPP)" components if you plan to deploy your Unity application to different operating systems.

## 3. Setting up a Unity Project for Robotics

1.  **Create a New Project**: In Unity Hub, go to the "Projects" tab and click "New Project". Select a 3D Core template.
2.  **Name and Location**: Give your project a meaningful name (e.g., `RoboticsSimulation`) and choose a suitable location.
3.  **Open Project**: Once created, open the project in the Unity Editor.

## 4. Integrating Unity with ROS 2

Unity does not natively communicate with ROS 2. You need specific packages that act as a bridge. The `Unity-Robotics-Hub` provides official tools for this.

1.  **Install Unity Robotics Hub**: Open your Unity project, navigate to `Window > Package Manager`. Select "Unity Registry" in the dropdown, search for "Robotics", and install `Robotics ROS-TCP Connector` and `Robotics ROS-TCP Endpoint` (if you plan to run the ROS 2 endpoint within Unity, which is common for standalone simulations).
    *Alternatively, you might find `ROS-Unity-Integration` or `RosMessageGeneration` packages depending on the Unity version and specific needs.*

2.  **Configure ROS Connection**: In your Unity scene, create an empty GameObject and rename it `ROSConnection`. Add the `ROSConnection` script component to it. In the Inspector, configure:
    *   **`Ros IP Address`**: The IP address of your ROS 2 machine (e.g., `127.0.0.1` for local, or the IP of your VM/Docker container).
    *   **`Ros Port`**: The port number (default is often `10000` or `5005` for `ROS-TCP-Connector`).

3.  **Generate ROS 2 Messages for Unity (if using custom messages)**:
    If your ROS 2 system uses custom message types, you'll need to generate corresponding C# scripts for Unity. The `Unity-Robotics-Hub` usually includes tools for this. Typically, you would:
    *   Copy your ROS 2 `.msg` files into a specific folder within your Unity project (e.g., `Assets/RosMessages/`).
    *   Use a provided Unity tool (e.g., `Robotics > ROS > Generate ROS Messages`) to convert them into C# scripts.

## 5. Basic Communication Example (Conceptual)

### Unity Publishing to ROS 2 (C# Script)

Create a C# script (e.g., `RobotPosePublisher.cs`) and attach it to a GameObject:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // For PoseStamped message

public class RobotPosePublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/robot_pose";
    public GameObject robotGameObject; // Assign your robot in Inspector

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    void Update()
    {
        if (robotGameObject != null)
        {
            PoseStampedMsg poseMsg = new PoseStampedMsg();
            poseMsg.header.stamp = ROSConnection.Get                  (this).TimeNow();
            poseMsg.header.frame_id = "map"; // Or your robot's base frame

            // Convert Unity's left-handed Z-up to ROS's right-handed Z-up
            poseMsg.pose.position.x = robotGameObject.transform.position.x;
            poseMsg.pose.position.y = robotGameObject.transform.position.z;
            poseMsg.pose.position.z = robotGameObject.transform.position.y; // Corrected for Z-up

            Quaternion rosRotation = new Quaternion(
                robotGameObject.transform.rotation.x,
                robotGameObject.transform.rotation.z, // Z and Y swapped
                robotGameObject.transform.rotation.y,
                robotGameObject.transform.rotation.w
            );
            poseMsg.pose.orientation.x = rosRotation.x;
            poseMsg.pose.orientation.y = rosRotation.y;
            poseMsg.pose.orientation.z = rosRotation.z;
            poseMsg.pose.orientation.w = rosRotation.w;

            ros.Publish(topicName, poseMsg);
        }
    }
}
```

### Unity Subscribing to ROS 2 (C# Script)

Create a C# script (e.g., `RobotVelocitySubscriber.cs`) and attach it to a GameObject:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry; // For Twist message

public class RobotVelocitySubscriber : MonoBehaviour
{
    public string topicName = "/cmd_vel";
    public float moveSpeed = 5f;
    public float turnSpeed = 100f;

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<TwistMsg>(topicName, ApplyVelocity);
    }

    void ApplyVelocity(TwistMsg twistMessage)
    {
        // Assuming a differential drive robot or similar base control
        float linearX = (float)twistMessage.linear.x;
        float angularZ = (float)twistMessage.angular.z;

        // Apply to the GameObject
        transform.Translate(Vector3.forward * linearX * moveSpeed * Time.deltaTime);
        transform.Rotate(Vector3.up, angularZ * turnSpeed * Time.deltaTime);
    }
}
```

## 6. Advanced Visualization and HRI

*   **Importing 3D Models**: Import URDF or FBX robot models into Unity. Use `Unity-Robotics-Hub` tools to convert URDF to Unity prefabs.
*   **Environment Design**: Build detailed indoor or outdoor environments using Unity's terrain tools, ProBuilder, and asset store models.
*   **UI/UX Development**: Utilize Unity's UI system (Canvas, UI Elements) to create dashboards, control panels, and interactive visualizations of robot state and sensor data.
*   **Camera Control**: Implement advanced camera systems (e.g., third-person, first-person, bird's-eye view) to monitor the robot and environment.

By following this guide, you will establish a robust Unity development environment for creating compelling and functional digital twins for physical AI and humanoid robotics.