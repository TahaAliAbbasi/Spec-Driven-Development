---
title: Capstone Project Guidelines and Rubrics
sidebar_position: 2
---

# Capstone Project Guidelines and Rubrics

## Project Overview

The Capstone Project requires students to develop a complete humanoid robot control system that can interpret a natural language command, plan a path, navigate an environment, identify an object using computer vision, and manipulate it. This project integrates concepts from all previous modules.

### Core Requirements

1.  **Natural Language Command Processing**:
    *   The robot must receive a simple, natural language command (e.g., "Go to the kitchen, find the red apple, and pick it up").
    *   Use OpenAI Whisper (or a similar ASR system) to convert speech to text, or accept text input directly for this project.
    *   Use an LLM (e.g., OpenAI GPT, or a local LLM) for cognitive planning to break down the command into a sequence of ROS 2 actions.

2.  **Robot Platform**:
    *   Use a humanoid robot model in simulation (Gazebo, Unity, or Isaac Sim). You may use a pre-existing model or create a simple one.
    *   The robot must be capable of navigation and manipulation (e.g., moving arms, grasping).

3.  **Navigation**:
    *   Implement path planning and navigation to move the robot to specified locations using Nav2.
    *   The robot must be able to avoid static and dynamic obstacles in the environment.

4.  **Perception**:
    *   Implement computer vision to identify and locate specific objects (e.g., a red apple) in the environment.
    *   Use simulated camera data from the simulation environment.

5.  **Manipulation**:
    *   Implement a basic manipulation task (e.g., picking up the identified object).
    *   Use ROS 2 control interfaces to command the robot's actuators.

6.  **Simulation Environment**:
    *   The entire project must be demonstrated in a simulated environment (Gazebo, Unity, or Isaac Sim).
    *   The environment should contain static obstacles, dynamic obstacles (optional), and the target objects.

### Project Deliverables

1.  **Codebase**: A well-documented ROS 2 workspace containing all necessary packages, launch files, and configuration files to run the project.
2.  **Documentation**:
    *   A `README.md` file explaining how to set up and run the project.
    *   A brief report (2-3 pages) describing the architecture, challenges faced, and solutions implemented.
3.  **Video Demonstration**: A short video (2-3 minutes) showing the robot successfully completing the task or explaining the most complex part of the implementation if the full task is not completed.
4.  **Presentation** (Optional for self-study): A presentation summarizing the project, including the approach, results, and lessons learned.

## Rubrics

Projects will be evaluated based on the following criteria:

### Functionality (40 points)

*   **Task Completion (20 points)**:
    *   20: The robot successfully completes the full task (NL command -> Nav -> Perceive -> Manipulate) in simulation.
    *   15: The robot completes most of the task (e.g., navigates and perceives, but manipulation fails).
    *   10: The robot completes a significant part of the task (e.g., navigation only).
    *   5: The robot demonstrates basic functionality (e.g., responds to a command but fails at navigation).
    *   0: The robot does not demonstrate the required functionality.

*   **Component Integration (20 points)**:
    *   20: All components (NL processing, LLM, Nav2, perception, manipulation) are well-integrated and communicate effectively via ROS 2.
    *   15: Most components are integrated, with minor communication issues or workarounds.
    *   10: Basic integration achieved, but with notable communication problems or manual steps.
    *   5: Some integration present, but significant parts operate independently.
    *   0: No meaningful integration of components.

### Technical Implementation (30 points)

*   **ROS 2 Usage (10 points)**:
    *   10: Correct and effective use of ROS 2 concepts (nodes, topics, services, actions, launch files, parameters).
    *   8: Mostly correct usage with minor issues.
    *   6: Basic ROS 2 concepts used, but with some incorrect patterns.
    *   4: Limited ROS 2 usage or incorrect patterns.
    *   0: ROS 2 not used correctly or not used at all.

*   **Perception and Planning (10 points)**:
    *   10: Sophisticated and robust implementation of perception (object detection/recognition) and path planning (Nav2 configuration, obstacle avoidance).
    *   8: Good implementation with minor limitations.
    *   6: Basic implementation that works under ideal conditions.
    *   4: Simple or flawed implementation.
    *   0: Not implemented or non-functional.

*   **Code Quality (10 points)**:
    *   10: Clean, well-documented, modular, and maintainable code.
    *   8: Good code with minor issues in style or documentation.
    *   6: Adequate code with some readability or documentation issues.
    *   4: Code is functional but poorly structured or documented.
    *   0: Code is messy, undocumented, or non-functional.

### Innovation and Problem-Solving (20 points)

*   **Creative Solutions (10 points)**:
    *   10: Demonstrates creative and innovative approaches to challenges.
    *   8: Shows some creative thinking in solving problems.
    *   6: Standard solutions applied effectively.
    *   4: Standard solutions with minor adaptations.
    *   0: No evidence of creative problem-solving.

*   **Challenge Handling (10 points)**:
    *   10: Successfully identifies and overcomes complex technical challenges.
    *   8: Addresses most challenges effectively.
    *   6: Handles basic challenges, with some unresolved issues.
    *   4: Attempts to address challenges but with limited success.
    *   0: Does not address any significant challenges.

### Documentation and Presentation (10 points)

*   **Documentation Clarity (5 points)**:
    *   5: Clear, comprehensive, and easy-to-follow documentation.
    *   4: Good documentation with minor omissions or unclear parts.
    *   3: Adequate documentation.
    *   2: Basic documentation with significant gaps.
    *   0: Poor or missing documentation.

*   **Video/Presentation Quality (5 points)**:
    *   5: Clear, engaging, and informative video or presentation.
    *   4: Good quality with minor issues.
    *   3: Adequate quality.
    *   2: Basic quality, hard to follow.
    *   0: Poor or missing video/presentation.

## Troubleshooting Common Integration Issues

This section will be expanded based on common problems encountered during project development. For now, consider these general tips:

*   **ROS 2 Communication**: Ensure all nodes are on the same `ROS_DOMAIN_ID`. Use `ros2 topic list` and `ros2 node list` to verify connectivity.
*   **Simulation-ROS Bridge**: If using Gazebo or Isaac Sim, ensure the ROS 2 bridge is correctly configured and running. Check for plugin errors in the simulation log.
*   **LLM/ASR Integration**: Handle API rate limits and errors gracefully. Consider caching or local models if internet access is unreliable.
*   **Performance**: Monitor CPU/GPU usage. Complex perception or planning algorithms might slow down the simulation.
*   **Object Detection**: Ensure your training data (or pre-trained models) are suitable for the objects in your simulation environment.
*   **Manipulation**: Start simple (e.g., moving an end-effector to a point) and gradually increase complexity (e.g., grasping).

## Sample Solutions and Starting Templates

This section will provide optional starting points and example architectures for the project. Students are encouraged to develop their own solutions but can use these as a reference if needed.

*   **Basic Architecture**: A suggested node structure (e.g., `nl_processor`, `task_planner`, `nav2_client`, `perception_node`, `manipulation_controller`).
*   **ROS 2 Launch File**: An example launch file to start all necessary nodes for the project.
*   **Simple Manipulation Example**: Basic code to move a robot arm to a specific pose using ROS 2 control interfaces.