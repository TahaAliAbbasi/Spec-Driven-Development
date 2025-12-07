---
title: "Module 1: Exercises and Quizzes"
sidebar_position: 4
---

# Module 1: Exercises and Quizzes

This section provides interactive exercises and quizzes to test your understanding of the concepts covered in Module 1: The Robotic Nervous System (ROS 2).

## Exercises

### Exercise 1.1: ROS 2 Publisher-Subscriber

**Goal**: Create a simple ROS 2 publisher and subscriber using `rclpy`.

1.  **Create a new ROS 2 package**: In your ROS 2 workspace, create a new Python package named `my_ros2_pkg`.
    ```bash
    ros2 pkg create --build-type ament_python my_ros2_pkg
    ```
2.  **Publisher Node**: Write a Python script (`my_ros2_pkg/my_ros2_pkg/publisher_node.py`) that publishes `std_msgs.msg.String` messages to a topic named `/chatter` at a rate of 1 Hz. The message data should be "Hello from Publisher: <count />".
3.  **Subscriber Node**: Write another Python script (`my_ros2_pkg/my_ros2_pkg/subscriber_node.py`) that subscribes to the `/chatter` topic and prints the received messages to the console.
4.  **Update `setup.py`**: Ensure your `setup.py` file correctly declares the entry points for both nodes.
5.  **Build and Run**: Build your ROS 2 workspace and run both nodes. Verify that the subscriber node is receiving and printing messages from the publisher.
    ```bash
    cd <ros2_workspace>
    colcon build
    source install/setup.bash
    ros2 run my_ros2_pkg publisher_node
    ros2 run my_ros2_pkg subscriber_node
    ```

### Exercise 1.2: ROS 2 Service Client-Server

**Goal**: Implement a simple ROS 2 service client and server using `rclpy`.

1.  **Service Definition**: Define a custom ROS 2 service message (e.g., `AddTwoInts.srv` in `my_ros2_pkg/srv/`) that takes two `int64` arguments (`a`, `b`) and returns one `int64` sum (`result`).
2.  **Service Server**: Write a Python script (`my_ros2_pkg/my_ros2_pkg/add_server.py`) that implements this service. It should receive two integers, sum them, and return the result.
3.  **Service Client**: Write another Python script (`my_ros2_pkg/my_ros2_pkg/add_client.py`) that acts as a client. It should call the `add_two_ints` service with two numbers and print the returned sum.
4.  **Update `setup.py` and `package.xml`**: Modify these files to include your custom service definition.
5.  **Build and Run**: Build your ROS 2 workspace and run the service server. Then, run the client to call the service. Verify the client receives the correct sum.
    ```bash
    cd <ros2_workspace>
    colcon build
    source install/setup.bash
    ros2 run my_ros2_pkg add_server
    # In a new terminal
    ros2 run my_ros2_pkg add_client
    ```

## Quizzes

### Quiz 1.1: ROS 2 Concepts

1.  What is the primary communication mechanism in ROS 2 for asynchronous, many-to-many data exchange?
    a) Services
    b) Actions
    c) Topics
    d) Parameters

2.  Which `rclpy` method is used to create a publisher?
    a) `create_subscriber()`
    b) `create_publisher()`
    c) `create_client()`
    d) `create_service()`

3.  What does URDF stand for?
    a) Universal Robot Data Format
    b) Unified Robotic Description File
    c) Unified Robot Description Format
    d) Universal Robotics Definition Framework

4.  Which ROS 2 command is used to inspect messages being published on a topic?
    a) `ros2 topic list`
    b) `ros2 run`
    c) `ros2 topic echo`
    d) `ros2 interface show`

### Quiz 1.2: `rclpy` and Python Agents

1.  What is the main advantage of using `rclpy` for AI agent integration in ROS 2?
    a) Better performance than C++
    b) Access to Python's rich AI/ML ecosystem
    c) Simpler syntax for low-level hardware control
    d) Native support for real-time operations

2.  Which ROS 2 package is typically used to broadcast a robot's kinematic state based on its URDF and joint states?
    a) `tf2_ros`
    b) `joint_state_publisher`
    c) `robot_state_publisher`
    d) `urdf_parser_py`

**Answers**:

*   Quiz 1.1: 1. c, 2. b, 3. c, 4. c
*   Quiz 1.2: 1. b, 2. c

---

These exercises and quizzes are designed to reinforce your understanding of Module 1's content and prepare you for subsequent modules.
