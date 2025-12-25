from omni.isaac.kit import SimulationApp

# Start Isaac Sim in headless mode for faster execution, or False for GUI
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np
import time

class CustomRobotControl:
    def __init__(self, asset_path):
        self.world = World(stage_units_in_meters=1.0) # 1.0 means 1 unit in USD is 1 meter
        self.world.scene.add_default_ground_plane()

        # Add custom robot to the stage
        self.robot_prim_path = "/World/CustomRobot"
        add_reference_to_stage(asset_path, self.robot_prim_path)

        # Add articulation (robot with joints)
        self.robot = self.world.scene.add(
            Articulation(
                prim_path=self.robot_prim_path,
                name="my_custom_robot",
                position=np.array([0.0, 0.0, 0.5]) # Adjust initial position if needed
            )
        )

        self.world.reset()
        self.timeline = omni.timeline.get_timeline_interface()

    def run_simulation(self):
        self.timeline.play()
        self.world.render()

        print(f"Robot name: {self.robot.name}")
        print(f"Number of joints: {self.robot.num_joints}")
        print(f"Joint names: {self.robot.get_joint_names()}")

        # Simple control loop: move first joint back and forth
        initial_joint_positions = self.robot.get_joint_positions()
        if self.robot.num_joints > 0:
            joint_index = 0 # Control the first joint
            joint_name = self.robot.get_joint_names()[joint_index]

            for _ in range(200): # Run for a duration
                self.world.step(render=True)
                current_time = self.world.get_simulation_time()

                # Oscillate joint position between -0.5 and 0.5 radians
                target_position = 0.5 * np.sin(current_time * 2.0)

                # Create an array for all joint positions, only changing the target joint
                target_positions = np.array(initial_joint_positions)
                target_positions[joint_index] = target_position

                self.robot.set_joint_positions(target_positions)
                print(f"Time: {current_time:.2f}, Joint '{joint_name}' target: {target_position:.2f}")

        else:
            print("No joints found for this robot. Skipping joint control example.")

        self.timeline.stop()
        simulation_app.close()

# Path to your custom robot USD asset
# IMPORTANT: Replace with the actual path to your robot model (e.g., from Nucleus or local path)
# Example Nucleus path:
# assets_root_path = get_assets_root_path()
# robot_usd_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"

# Example local path:
robot_usd_path = "/path/to/your/custom_robot.usd" # <--- REPLACE THIS WITH YOUR ROBOT'S USD PATH

if __name__ == "__main__":
    # Ensure you replace the 'robot_usd_path' above with a valid path to your custom robot model.
    # For instance, if you have a robot at 'C:/Users/YourUser/Documents/my_robot.usd',
    # set robot_usd_path = "C:/Users/YourUser/Documents/my_robot.usd"
    # If using a built-in Isaac Sim asset, use get_assets_root_path() to construct the path.

    print("Please update the 'robot_usd_path' variable in this script with the actual path to your robot USD file.")
    print("Example: robot_usd_path = \"omniverse://localhost/NVIDIA/Assets/Isaac/2023.1.1/Robots/Franka/franka.usd\"")
    print("Or for a local file: robot_usd_path = \"C:/path/to/my_robot.usd\"")

    # Only run if a valid path is provided by the user
    # if robot_usd_path != "/path/to/your/custom_robot.usd":
    #     controller = CustomRobotControl(robot_usd_path)
    #     controller.run_simulation()
    # else:
    #     print("Skipping simulation as no valid robot USD path was provided.")

    # For now, just create the instance without running the simulation
    # Users will need to manually update the path and uncomment the execution block.
    print("Script generated. Please update the 'robot_usd_path' and uncomment the execution block to run.")

