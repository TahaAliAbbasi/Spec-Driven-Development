---
title: Capstone Project Troubleshooting Guide
sidebar_position: 3
---

# Capstone Project Troubleshooting Guide

This guide addresses common issues encountered when integrating multiple complex systems (ROS 2, simulation, AI models) for the Capstone Project. Use this as a reference to diagnose and resolve problems.

## General ROS 2 Communication Issues

### Symptoms
*   Nodes cannot communicate (topics/services not connecting).
*   Error messages like "Failed to contact master" or "No route to host" (if using multi-machine setup).
*   `ros2 topic list` or `ros2 node list` shows fewer items than expected.

### Solutions
1.  **Check `ROS_DOMAIN_ID`**: Ensure all terminals sourcing your ROS 2 environment (and potentially Isaac Sim if it bridges to ROS 2) are using the same `ROS_DOMAIN_ID`. Default is 0.
    ```bash
echo $ROS_DOMAIN_ID # Check current ID
export ROS_DOMAIN_ID=0 # Set to 0 (or desired ID) in your terminal session
    ```
2.  **Verify Network Configuration**: If running parts of your system on different machines or in different Docker containers, ensure network settings allow discovery (e.g., using `--network=host` in Docker, or configuring `RMW_IMPLEMENTATION` and `ROS_LOCALHOST_ONLY`).
3.  **Check Firewall**: Ensure firewall rules are not blocking DDS (Data Distribution Service) traffic used by ROS 2. Ports typically range from 7400 upwards.
4.  **Inspect Logs**: Check the console output of your ROS 2 nodes for specific error messages. Use `ros2 run <pkg_name> <node_name> --ros-args --log-level debug` for more verbose logging.

## Simulation (Gazebo/Isaac Sim/Unity) - ROS 2 Bridge Issues

### Symptoms
*   Simulation does not respond to ROS 2 commands.
*   Sensor data (e.g., camera images, LiDAR) is not published to ROS 2 topics from the simulator.
*   Robot model does not appear correctly in simulation or control is erratic.

### Solutions
1.  **Verify Plugin Installation**: Ensure the necessary ROS 2 bridge plugins are installed and loaded.
    *   **Gazebo**: Check for `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`, `libgazebo_ros_p3d.so`, etc. Ensure they are loaded via `<plugin>` tags in your world or model files, or via the `gzserver` command-line arguments.
    *   **Isaac Sim**: Ensure the `omni.isaac.ros2_bridge` extension is enabled in Isaac Sim. Check the USD stage for ROS 2 components (e.g., `ROS2Camera`, `ROS2DiffDrive`) attached to relevant prims.
    *   **Unity**: Ensure the ROS-TCP-Connector or ROS-Unity-Integration package is correctly configured in your Unity project, and the `ROSConnection` GameObject is present and configured with the correct IP/Port.
2.  **Check Robot Model Configuration**: For Gazebo/Isaac Sim, ensure your robot's URDF/SDF/USD correctly defines joints and links, and that any ROS 2 control plugins are correctly specified. For Isaac Sim, ensure the robot is an `Articulation` and ROS 2 components are attached to the correct prims.
3.  **Verify Topic Names**: Ensure the topic names published by the simulation (e.g., `/camera/image_raw`, `/scan`, `/cmd_vel`) match the topic names expected by your ROS 2 nodes. Use `ros2 topic list` and `ros2 topic echo <topic_name>` to verify.
4.  **Simulation Timing**: If control is erratic, check the simulation's update rate and ensure your ROS 2 control loop frequency is appropriate.

## LLM/Natural Language Processing Integration Issues

### Symptoms
*   LLM does not respond or gives irrelevant responses.
*   High latency in processing natural language commands.
*   API errors (e.g., rate limits, authentication).

### Solutions
1.  **API Keys and Configuration**: Double-check that your OpenAI API key (or keys for other LLMs) is correctly set as an environment variable (`OPENAI_API_KEY`) or loaded from a secure configuration file.
2.  **Handle Rate Limits**: Implement retry logic with exponential backoff for API calls. Consider caching responses for common commands.
    ```python
import openai
import time
import backoff # pip install backoff

@backoff.on_exception(backoff.expo, openai.RateLimitError)
def get_completion_with_backoff(**kwargs):
    return openai.ChatCompletion.create(**kwargs)

# Use get_completion_with_backoff instead of openai.ChatCompletion.create
    ```
3.  **Prompt Engineering**: Refine your prompts to the LLM to be more specific and structured. For example, provide a clear context ("You are controlling a humanoid robot..."), the input ("The user command is: ..."), and the desired output format ("Respond with a JSON list of actions: ...").
4.  **Local Models**: If latency or API costs are prohibitive, consider using local LLMs (e.g., via `llama.cpp`, `transformers` with a downloaded model) or smaller, faster models for the cognitive planning step.

## Computer Vision (Object Detection) Issues

### Symptoms
*   Object not detected in the camera image.
*   Incorrect object localization (bounding box/coordinates are wrong).
*   High latency in perception pipeline.

### Solutions
1.  **Check Camera Data**: Verify that your ROS 2 node is correctly subscribing to the camera topic (e.g., `/camera/rgb/image_raw`) and receiving images. Use `ros2 run image_view image_view` to visualize the topic directly.
2.  **Model Suitability**: Ensure your object detection model is trained on or suitable for the objects in your simulation environment. A model trained on real-world images might not perform well on synthetic data without fine-tuning or domain adaptation techniques.
3.  **Image Preprocessing**: Check that the image format, resolution, and color space (e.g., BGR vs RGB) match the expectations of your detection model.
4.  **Coordinate Frame Transformation**: After detecting an object in the camera frame, you often need to transform its 2D pixel coordinates to 3D world coordinates (e.g., using depth information or camera intrinsics/extrinsics) to plan manipulation.
5.  **Performance**: If using a heavy model, consider running the detection node on a separate thread or at a lower frequency. Optimize the model (e.g., quantization) if possible.

## Navigation (Nav2) Issues

### Symptoms
*   Robot fails to plan a path.
*   Robot gets stuck or oscillates.
*   Robot fails to avoid obstacles.

### Solutions
1.  **Map and Localization**: Ensure you have a correct map and the robot is accurately localized within it (check `/amcl_pose` or use `rviz2`).
2.  **Costmap Configuration**: Review your `costmap_2d` parameters. Ensure `obstacle_range`, `raytrace_range`, `inflation_radius`, etc., are tuned for your robot and environment.
3.  **Planner Configuration**: The default `NavFn` global planner or `DWB` local planner might not be optimal. Experiment with alternatives like `GlobalPlanner`, `TEBPlanner`, or `Smoother` if available.
4.  **Robot Footprint**: Ensure the robot's footprint (`robot_radius` or `footprint` parameter in costmap config) accurately reflects its physical size for collision avoidance.
5.  **TF Tree**: Verify the TF tree is correct and complete (e.g., `map` -> `odom` -> `base_link` -> `camera`, `laser`). Use `ros2 run tf2_tools view_frames` and `ros2 run tf2_ros tf2_echo <frame1> <frame2>`.

## Manipulation Issues

### Symptoms
*   Robot arm does not move as commanded.
*   Grasping fails repeatedly.
*   Inverse Kinematics (IK) solver fails.

### Solutions
1.  **Control Interface**: Ensure you are using the correct control interface (e.g., joint trajectory controller, MoveIt! motion planning). Check the controller manager status with `ros2 control list_controllers`.
2.  **Joint Limits**: Verify that commanded joint positions are within the physical limits defined in your robot's URDF/SDF.
3.  **Grasping Strategy**: Grasping is a complex task. Start with simple, geometrically graspable objects. Consider using a pre-defined grasp generator or a learned grasp model if available.
4.  **Simulation Physics**: In Gazebo, ensure the `ODE` or `Bullet` physics parameters for the object and gripper are realistic (mass, friction, stiffness) to allow for stable grasping. In Isaac Sim, check PhysX settings.
5.  **End-Effector Alignment**: Precise positioning and orientation of the end-effector relative to the object are crucial for successful grasping. Use MoveIt! or a custom IK solver to calculate joint angles for a desired end-effector pose.

## Performance Issues

### Symptoms
*   Overall system runs slowly.
*   Simulation time factor is low (< 1.0).
*   High CPU/GPU usage.

### Solutions
1.  **Simulation Settings**: In Gazebo/Isaac Sim, adjust physics update rate, rendering quality (in Isaac Sim), or disable unnecessary plugins/rendering if real-time performance is not critical for development.
2.  **LLM/Perception Frequency**: Do not run heavy AI models (LLM calls, object detection) at the maximum simulation frequency. Use a lower frequency (e.g., 1-5 Hz) or trigger them based on specific events.
3.  **ROS 2 Node Frequency**: Ensure your control nodes are running at appropriate frequencies. Overly frequent publishing/subscribing can create unnecessary overhead.
4.  **Hardware**: Ensure your development machine meets the minimum requirements, especially for Isaac Sim (NVIDIA GPU with RTX) and running complex perception models.

By systematically checking these areas, you should be able to identify and resolve most integration challenges in your Capstone Project.