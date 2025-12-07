---
title: Capstone Project Sample Solution Template
sidebar_position: 4
---

# Capstone Project Sample Solution Template

This document provides a basic template and example architecture for the Capstone Project. It demonstrates how to structure your ROS 2 workspace and integrate the key components. This is *one possible approach* and students are encouraged to adapt and expand upon it.

## Project Structure

Your ROS 2 workspace should follow a standard structure:

```
capstone_ws/
├── src/
│   ├── capstone_bringup/         # Launch files to start the entire system
│   │   ├── launch/
│   │   │   └── capstone_demo.launch.py
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   ├── nl_command_interpreter/   # Node for processing natural language
│   │   ├── src/
│   │   │   └── nl_interpreter_node.py
│   │   ├── config/
│   │   │   └── openai_params.yaml
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   ├── task_planner/             # Node for cognitive planning with LLM
│   │   ├── src/
│   │   │   └── task_planner_node.py
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   ├── perception_node/          # Node for object detection
│   │   ├── src/
│   │   │   └── simple_detector.py
│   │   └── CMakeLists.txt
│   │   └── package.xml
│   └── manipulation_controller/  # Node for basic manipulation
│       ├── src/
│       │   └── simple_manipulator.py
│       └── CMakeLists.txt
│       └── package.xml
```

## Example Components

### 1. Launch File (`capstone_bringup/launch/capstone_demo.launch.py`)

This file starts all necessary nodes for the demo.

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to config file (if needed)
    # config = os.path.join(get_package_share_directory('nl_command_interpreter'), 'config', 'openai_params.yaml')

    return LaunchDescription([
        # Node for interpreting natural language commands
        Node(
            package='nl_command_interpreter',
            executable='nl_interpreter_node',
            name='nl_interpreter',
            # parameters=[config] # Uncomment if using a config file
        ),

        # Node for task planning using LLM
        Node(
            package='task_planner',
            executable='task_planner_node',
            name='task_planner',
        ),

        # Node for perception (object detection)
        Node(
            package='perception_node',
            executable='simple_detector',
            name='simple_detector',
            # Example remapping: ROS2 topic from simulation to your node's subscription
            remappings=[('/camera/image_raw', '/your_sim_camera_topic')]
        ),

        # Node for manipulation
        Node(
            package='manipulation_controller',
            executable='simple_manipulator',
            name='simple_manipulator',
        ),

        # Example: Start Nav2 (assuming it's already configured for your robot/map)
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager',
        #     parameters=[{'use_sim_time': True}] # Important if using sim time
        # ),
        # Add other Nav2 nodes (amcl, planner_server, controller_server, etc.) as needed based on your Nav2 setup.
    ])

```

### 2. Natural Language Interpreter Node (`nl_command_interpreter/src/nl_interpreter_node.py`)

This is a conceptual example. It might receive a text command and publish it to a planning node.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai # Requires 'pip install openai'

class NLInterpreterNode(Node):
    def __init__(self):
        super().__init__('nl_interpreter_node')
        # Publisher to send processed command to the task planner
        self.planner_publisher = self.create_publisher(String, 'planning/command', 10)

        # For this example, we'll simulate receiving a command via a timer
        # In practice, this might come from Whisper via another node or a service call.
        self.i = 0
        self.timer = self.create_timer(5.0, self.timer_callback) # Publish every 5 seconds

    def timer_callback(self):
        # Example command
        command_text = "Go to the red cube and pick it up."
        msg = String()
        msg.data = command_text
        self.planner_publisher.publish(msg)
        self.get_logger().info(f'Publishing command: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = NLInterpreterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 3. Task Planner Node (`task_planner/src/task_planner_node.py`)

This node receives the command, uses an LLM to break it down, and might publish a sequence of actions.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import openai # Requires 'pip install openai'

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')
        # Subscriber to receive command from NL interpreter
        self.command_subscriber = self.create_subscription(
            String,
            'planning/command',
            self.command_callback,
            10
        )

        # Publisher to send action sequence (could be a custom message type)
        self.action_publisher = self.create_publisher(String, 'planning/action_sequence', 10)

    def command_callback(self, msg):
        self.get_logger().info(f'Received command: "{msg.data}"')
        # Call LLM to parse command into actions
        actions = self.call_llm_for_planning(msg.data)
        if actions:
            action_msg = String()
            action_msg.data = json.dumps(actions) # Serialize list of actions
            self.action_publisher.publish(action_msg)
            self.get_logger().info(f'Published action sequence: {actions}')

    def call_llm_for_planning(self, command):
        # Example prompt for the LLM
        prompt = f"""
        You are a helpful assistant for a robotics project. Given a natural language command for a humanoid robot, please break it down into a sequence of simple, executable actions. Respond with a JSON list of actions.

        Command: "{command}"

        Example output format:
        [
            {{"action": "navigate_to", "target": "kitchen"}},
            {{"action": "detect_object", "object_type": "red apple"}},
            {{"action": "move_arm_to_object", "object_position": "..."}},
            {{"action": "grasp_object"}}
        ]

        Please provide the action sequence now.
        """

        try:
            # Make the API call (ensure OPENAI_API_KEY is set in environment)
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", # Or another suitable model
                messages=[{"role": "user", "content": prompt}],
                max_tokens=200, # Adjust as needed
                temperature=0.1, # Low temperature for more deterministic output
            )
            # Parse the response
            content = response.choices[0].message['content'].strip()
            # Attempt to parse JSON from the LLM response
            import re
            json_match = re.search(r'\[.*\]', content, re.DOTALL) # Find JSON array
            if json_match:
                actions = json.loads(json_match.group(0))
                return actions
            else:
                self.get_logger().error(f'Could not parse JSON from LLM response: {content}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error calling LLM: {e}')
            return None

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 4. Perception Node (`perception_node/src/simple_detector.py`)

This node subscribes to a camera topic and performs basic object detection (using a placeholder logic here).

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # Could be a custom message for detection results
import numpy as np
# Example: import your detection library (e.g., OpenCV, PyTorch, etc.)
# import cv2
# import torch

class SimpleDetectorNode(Node):
    def __init__(self):
        super().__init__('simple_detector')
        # Subscriber to camera image
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', # Ensure this matches your sim camera topic
            self.image_callback,
            10
        )
        # Publisher for detection results
        self.detection_publisher = self.create_publisher(String, 'perception/detections', 10)

    def image_callback(self, msg):
        # self.get_logger().info('Received an image')
        # --- PLACEHOLDER FOR ACTUAL DETECTION LOGIC ---
        # Convert sensor_msgs/Image to OpenCV/Numpy format if needed
        # raw_data = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        # results = your_detection_model(raw_data) # e.g., using YOLO, Detectron2, etc.

        # For this template, we'll simulate a detection result
        detection_result = {
            "object_type": "red_cube", # Example detected object
            "confidence": 0.95,
            "bbox": {"x_min": 100, "y_min": 150, "x_max": 200, "y_max": 250},
            "position_3d": {"x": 1.0, "y": 2.0, "z": 0.5} # Example 3D position (requires depth/TF)
        }

        detection_msg = String()
        detection_msg.data = str(detection_result) # Or use a custom message type
        self.detection_publisher.publish(detection_msg)
        self.get_logger().info(f'Published detection: {detection_result}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

### 5. Manipulation Controller Node (`manipulation_controller/src/simple_manipulator.py`)

This node might receive commands to move the robot's arm or gripper.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Could be a custom message for manipulation goals
# Example: Import messages for MoveIt! or joint trajectory control
# from moveit_msgs.msg import MoveGroupGoal
# from trajectory_msgs.msg import JointTrajectory

class SimpleManipulatorNode(Node):
    def __init__(self):
        super().__init__('simple_manipulator')
        # Subscriber for manipulation commands (could be from task planner or perception)
        self.manipulation_subscriber = self.create_subscription(
            String,
            'manipulation/command', # Example topic
            self.manipulation_callback,
            10
        )
        # Example: Publisher for joint trajectory commands
        # self.trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

    def manipulation_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'Received manipulation command: {command}')
        # --- PLACEHOLDER FOR ACTUAL MANIPULATION LOGIC ---
        # Parse command (e.g., "grasp_object_at 1.0 2.0 0.5")
        # Use MoveIt! Python interface or publish JointTrajectory messages
        # Example pseudo-code:
        # if "grasp" in command:
        #     self.execute_grasp()
        # elif "move_to" in command:
        #     target_pos = parse_position(command)
        #     self.move_arm_to(target_pos)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleManipulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

```

## Building and Running

1.  **Source ROS 2**: Ensure your ROS 2 environment is sourced.
    ```bash
source /opt/ros/humble/setup.bash # Or your ROS 2 distro
    ```
2.  **Build the Workspace**:
    ```bash
cd /path/to/capstone_ws
colcon build --packages-select capstone_bringup nl_command_interpreter task_planner perception_node manipulation_controller
    ```
3.  **Source the Workspace**:
    ```bash
source install/setup.bash
    ```
4.  **Run the Launch File**:
    ```bash
ros2 launch capstone_bringup capstone_demo.launch.py
    ```
    Ensure your simulation environment (Gazebo/Isaac Sim/Unity) is running and configured to interface with ROS 2 before launching these nodes.

This template provides a basic skeleton. You will need to fill in the "PLACEHOLDER" sections with actual logic, configure your specific robot and simulation environment, and potentially create custom ROS 2 message types for more complex data exchange between nodes.