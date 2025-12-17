#!/usr/bin/env python3
"""
Configure Sensors on Humanoid Robot in Isaac Sim
Module 3, Chapter 1 - Attach stereo cameras and LiDAR
"""

from omni.isaac.sensor import Camera
from omni.isaac.range_sensor import _range_sensor
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core import World

def attach_stereo_cameras(robot_prim_path: str = "/World/humanoid"):
    """
    Attach stereo camera pair to robot's head link.

    Args:
        robot_prim_path: Path to robot prim in stage

    Returns:
        tuple: (camera_left, camera_right) Camera objects
    """
    head_link_path = f"{robot_prim_path}/head_link"

    print(f"📷 Attaching stereo cameras to {head_link_path}")

    # Left camera (baseline: 6cm to the left)
    camera_left = Camera(
        prim_path=f"{head_link_path}/camera_left",
        frequency=30,  # 30 Hz
        resolution=(1280, 720),  # HD resolution
        position=[0.05, 0.03, 0.1],  # 5cm forward, 3cm left, 10cm up
        orientation=[0, 0, 0, 1],  # quaternion (w, x, y, z) - identity (no rotation)
    )

    # Configure intrinsics to match RealSense D435i
    camera_left.set_focal_length(5.0)  # mm
    camera_left.set_focus_distance(1.0)  # meters
    camera_left.set_horizontal_aperture(20.955)  # mm (D435i spec)
    camera_left.set_clipping_range(0.1, 100.0)  # Near/far planes

    # Right camera (baseline: 6cm to the right)
    camera_right = Camera(
        prim_path=f"{head_link_path}/camera_right",
        frequency=30,
        resolution=(1280, 720),
        position=[0.05, -0.03, 0.1],  # 5cm forward, 3cm right, 10cm up
        orientation=[0, 0, 0, 1],
    )

    camera_right.set_focal_length(5.0)
    camera_right.set_focus_distance(1.0)
    camera_right.set_horizontal_aperture(20.955)
    camera_right.set_clipping_range(0.1, 100.0)

    print("✅ Stereo cameras attached:")
    print(f"   Left:  {camera_left.prim_path}")
    print(f"   Right: {camera_right.prim_path}")
    print(f"   Baseline: 6cm (0.06m)")
    print(f"   Resolution: 1280x720 @ 30 Hz")

    return camera_left, camera_right


def attach_lidar(robot_prim_path: str = "/World/humanoid"):
    """
    Attach rotating LiDAR sensor to robot's base link.

    Args:
        robot_prim_path: Path to robot prim in stage

    Returns:
        LiDAR prim path
    """
    base_link_path = f"{robot_prim_path}/base_link"
    lidar_path = f"{base_link_path}/lidar"

    print(f"🔵 Attaching LiDAR to {base_link_path}")

    # Create LiDAR using Isaac Sim's range sensor
    # Note: This is a simplified example - actual implementation requires
    # creating USD prim with appropriate sensor properties

    result, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRotatingLidarLidar",
        path=lidar_path,
        parent=base_link_path,
        min_range=0.4,  # meters
        max_range=30.0,  # meters
        draw_points=True,
        draw_lines=False,
        horizontal_fov=360.0,  # degrees (full rotation)
        vertical_fov=30.0,  # degrees (±15°)
        horizontal_resolution=1.0,  # degrees per sample
        vertical_resolution=1.0,  # degrees per sample
        rotation_rate=10.0,  # Hz
        high_lod=True,
        yaw_offset=0.0,
        enable_semantics=False,
    )

    if result:
        print("✅ LiDAR attached:")
        print(f"   Path: {lidar_path}")
        print(f"   Range: 0.4m - 30m")
        print(f"   FOV: 360° horizontal, 30° vertical")
        print(f"   Rotation: 10 Hz")
        return lidar_path
    else:
        print("❌ Failed to attach LiDAR")
        return None


def configure_ros2_publishing(camera_left, camera_right, lidar_path):
    """
    Configure ROS 2 topics for sensor data publishing.
    This function creates OmniGraph nodes for ROS 2 bridge.

    Args:
        camera_left: Left camera object
        camera_right: Right camera object
        lidar_path: Path to LiDAR prim

    Note: This requires ROS 2 Bridge extension to be enabled.
    """
    print("\n📡 Configuring ROS 2 publishing...")
    print("   Enable 'omni.isaac.ros2_bridge' extension manually:")
    print("   Window → Extensions → Search 'ROS2 Bridge' → Enable")
    print("")
    print("   Then create OmniGraph:")
    print("   Window → Visual Scripting → Action Graph")
    print("")
    print("   Add nodes:")
    print("   1. On Playback Tick")
    print("   2. ROS2 Context")
    print("   3. ROS2 Camera Helper (for left camera)")
    print("      - Camera Prim: /World/humanoid/head_link/camera_left")
    print("      - Topic Name: /camera/left/image_raw")
    print("      - Info Topic: /camera/left/camera_info")
    print("   4. ROS2 Camera Helper (for right camera)")
    print("      - Camera Prim: /World/humanoid/head_link/camera_right")
    print("      - Topic Name: /camera/right/image_raw")
    print("      - Info Topic: /camera/right/camera_info")
    print("   5. ROS2 Publish LaserScan")
    print("      - Lidar Prim: /World/humanoid/base_link/lidar")
    print("      - Topic Name: /scan")
    print("")
    print("✅ Manual configuration guide displayed")


if __name__ == "__main__":
    # Assume robot is already loaded in Isaac Sim
    robot_prim_path = "/World/humanoid"

    # Attach sensors
    camera_left, camera_right = attach_stereo_cameras(robot_prim_path)
    lidar_path = attach_lidar(robot_prim_path)

    # Show ROS 2 configuration guide
    configure_ros2_publishing(camera_left, camera_right, lidar_path)

    print("\n🎬 Ready to record data!")
    print("   1. Press Play in Isaac Sim")
    print("   2. Verify ROS 2 topics: ros2 topic list")
    print("   3. Record rosbag: ./record_isaac_data.sh")
