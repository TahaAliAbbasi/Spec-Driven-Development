# Images for Physical AI & Humanoid Robotics Book

This directory contains images and diagrams referenced in the Physical AI & Humanoid Robotics documentation.

## Required Diagrams

The following diagrams should be created for the book:

### system-architecture.svg
- **Used in:** docs/intro.md
- **Description:** High-level architecture showing the integration of ROS 2, simulation environments, AI components, and robot platforms
- **Elements to include:**
  - User commands flowing to AI components (Whisper, LLMs)
  - AI components connecting to ROS 2 communication layer
  - ROS 2 connecting to simulation environments (Gazebo, Isaac Sim) and physical robots
  - Sensor data flowing back to AI components
  - Bidirectional arrows showing communication flow

### vla-process-flow.svg
- **Used in:** docs/module4/README.md
- **Description:** Flow diagram showing the Vision-Language-Action pipeline
- **Elements to include:**
  - Voice input at the start
  - Whisper processing to convert speech to text
  - LLM cognitive planning to generate action sequences
  - ROS 2 command execution
  - Robot action output
  - Visual perception feedback loop

### ros2-communication-patterns.svg
- **Used in:** docs/module1/chapter2.md
- **Description:** Diagram showing ROS 2 communication patterns
- **Elements to include:**
  - Nodes as boxes
  - Topics as arrows between nodes
  - Publishers and subscribers
  - Services and clients
  - Example message flows

### simulation-environment-integration.svg
- **Used in:** docs/module2/README.md, docs/module3/README.md
- **Description:** Showing how simulation environments integrate with ROS 2
- **Elements to include:**
  - Simulation environment (Gazebo/Unity/Isaac Sim)
  - ROS 2 bridge
  - Sensor data flow from simulation to ROS 2
  - Control command flow from ROS 2 to simulation

## Image Specifications

- All diagrams should be created as SVG files for scalability
- Use consistent color scheme throughout
- Include clear labels and legends where necessary
- Aim for clarity over artistic detail
- Keep file sizes reasonable while maintaining quality