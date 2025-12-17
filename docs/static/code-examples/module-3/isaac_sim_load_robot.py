#!/usr/bin/env python3
"""
Load Humanoid Robot URDF into Isaac Sim
Module 3, Chapter 1
"""

import omni
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.importer.urdf import _urdf
import sys

def load_robot_urdf(urdf_path: str, robot_name: str = "humanoid"):
    """
    Load a URDF file into Isaac Sim programmatically.

    Args:
        urdf_path: Absolute path to the URDF file
        robot_name: Name for the robot prim in the stage

    Returns:
        bool: True if import succeeded, False otherwise
    """
    # Enable URDF extension
    enable_extension("omni.importer.urdf")

    # Create simulation world
    world = World(stage_units_in_meters=1.0)

    # Configure URDF import settings
    robot_prim_path = f"/World/{robot_name}"

    importer = _urdf.acquire_urdf_interface()
    import_config = _urdf.ImportConfig()

    # Humanoid-specific settings
    import_config.merge_fixed_joints = False
    import_config.fix_base = False  # Floating base for humanoid (not fixed to ground)
    import_config.import_inertia_tensor = True
    import_config.self_collision = True  # Enable self-collision detection
    import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    import_config.default_position_drive_damping = 1000.0
    import_config.default_position_drive_stiffness = 10000.0

    # Import URDF
    print(f"📦 Importing URDF from: {urdf_path}")
    print(f"🎯 Target prim path: {robot_prim_path}")

    success = importer.parse_and_import_urdf(
        urdf_path, robot_prim_path, import_config
    )

    if success:
        print(f"✅ Robot '{robot_name}' loaded successfully at {robot_prim_path}")
        print(f"🎮 Use Play button to start physics simulation")

        # Optional: Play simulation automatically
        # world.play()

        return True
    else:
        print(f"❌ Failed to load URDF from {urdf_path}")
        print("💡 Check:")
        print("   - URDF file exists and is valid")
        print("   - All mesh files (.stl/.dae) are accessible")
        print("   - No syntax errors in URDF XML")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python isaac_sim_load_robot.py <path_to_urdf>")
        print("Example: python isaac_sim_load_robot.py /home/user/robot/humanoid.urdf")
        sys.exit(1)

    urdf_path = sys.argv[1]
    robot_name = sys.argv[2] if len(sys.argv) > 2 else "humanoid"

    load_robot_urdf(urdf_path, robot_name)
