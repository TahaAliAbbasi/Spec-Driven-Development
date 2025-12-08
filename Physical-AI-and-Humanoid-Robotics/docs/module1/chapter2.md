---
title: "Chapter 2: ROS 2 Basics - Learning Outcomes"
sidebar_position: 2
---

# Chapter 2: ROS 2 Basics - Learning Outcomes

This chapter introduces the fundamental concepts and building blocks of ROS 2. Upon completion, students will be able to:

*   **Explain ROS 2 Nodes**: Describe the role of nodes as executable processes in a ROS 2 system and how they communicate.
*   **Understand ROS 2 Topics**: Explain topics as the primary mechanism for asynchronous, many-to-many data exchange in ROS 2, including publisher and subscriber concepts.
*   **Grasp ROS 2 Services**: Describe services as a request-response communication mechanism for synchronous, one-to-one interactions.
*   **Identify ROS 2 Messages**: Recognize the importance of message types in defining data structures for topics and services.
*   **Utilize `ros2 run` and `ros2 topic` commands**: Execute basic ROS 2 commands for launching nodes and inspecting topic communication.
*   **Write simple Publisher/Subscriber Nodes**: Implement basic C++ or Python (using `rclpy`) nodes that publish and subscribe to topics.
*   **Implement simple Service Client/Server**: Develop basic C++ or Python (`rclpy`) nodes that provide and consume services.

These learning outcomes will guide the content and exercises for this chapter.

## Introduction to ROS 2 Communication

ROS 2 (Robotic Operating System 2) is a flexible framework for writing robot software. It provides a standardized way for different components of a robotics system to communicate and work together. At its core, ROS 2 is about distributed communication, enabling various software modules (nodes) to exchange information.

![ROS 2 Communication Patterns](/img/ros2-communication-patterns.png)
*Note: Flow Diagram showing ROS 2 communication patterns.*

### ROS 2 Nodes

A **node** is the fundamental unit of computation in ROS 2. It's an executable process that performs a specific task. A robot control system typically consists of many nodes, each responsible for a small, modular function (e.g., a camera driver, a motor controller, a navigation algorithm).

*   **Example**: A single robot might have nodes for:
    *   `camera_publisher_node`: Reads images from a camera and publishes them.
    *   `motor_controller_node`: Receives commands and sends signals to motors.
    *   `path_planner_node`: Computes a safe path for the robot.

Nodes are designed to be modular and reusable, allowing developers to build complex systems by combining simpler components.

### ROS 2 Topics

**Topics** are the most common way for nodes to exchange data in an asynchronous, many-to-many fashion. Data is published on a topic by a **publisher** node and can be received by one or more **subscriber** nodes.

*   **Key Characteristics**:
    *   **Asynchronous**: Publishers and subscribers don't wait for each other.
    *   **One-to-many**: A single publisher can send data to multiple subscribers.
    *   **Message-based**: Data is exchanged as structured messages.
    *   **Decoupled**: Publishers and subscribers don't need to know about each other's existence directly; they only need to agree on the topic name and message type.

*   **Analogy**: Think of a radio station (publisher) broadcasting information on a specific frequency (topic). Anyone with a radio tuned to that frequency (subscriber) can receive the information.

*   **Example**: A `camera_publisher_node` might publish image data on a `/robot/camera/image` topic, and a `vision_processing_node` and a `data_logger_node` could both subscribe to this topic to receive the images.

### ROS 2 Services

**Services** provide a synchronous, request-response communication mechanism between nodes. Unlike topics, which are asynchronous and one-to-many, services are blocking and one-to-one.

*   **Key Characteristics**:
    *   **Synchronous**: The client node sends a request and waits for the server node's response before continuing its execution.
    *   **One-to-one**: A client sends a request to a single service server.
    *   **Request/Response**: Defined by a service type with a request message and a response message.

*   **Analogy**: Think of calling a function. The caller (client) waits for the function (service server) to execute and return a result before proceeding.

*   **Example**: A `robot_arm_controller` node might offer a `/robot/arm/move_to_pose` service. A `task_planner_node` could be a client, sending a request with a desired arm pose and waiting for a response indicating success or failure of the movement.

### ROS 2 Messages

**Messages** are the data structures used for communication over topics and services. They are defined using a simple interface description language (IDL) and then code-generated into various programming languages (e.g., C++, Python).

*   **Example**: A simple message for a robot's `Pose` might look like:

    ```
    # Pose.msg
    geometry_msgs/Point position
    geometry_msgs/Quaternion orientation
    ```

    This defines a `Pose` message containing `position` (a `Point`) and `orientation` (a `Quaternion`), both of which are common ROS 2 message types.

### Basic ROS 2 Commands

Several command-line tools are essential for interacting with ROS 2 systems:

*   **`ros2 run <package_name> <executable_name>`**: Launches a ROS 2 node.
    *   Example: `ros2 run demo_nodes_cpp talker`
*   **`ros2 topic list`**: Lists all active topics in the ROS 2 graph.
*   **`ros2 topic echo <topic_name>`**: Displays messages being published on a topic.
    *   Example: `ros2 topic echo /robot/camera/image`
*   **`ros2 interface show <message_type>`**: Shows the definition of a ROS 2 message or service type.
    *   Example: `ros2 interface show geometry_msgs/msg/Point`
*   **`ros2 service list`**: Lists all active services.
*   **`ros2 service call <service_name> <service_type> <request_args>`**: Calls a service with specific arguments.
    *   Example: `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"`

These commands provide powerful introspection and interaction capabilities for debugging and monitoring ROS 2 applications.
