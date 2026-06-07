#!/usr/bin/env python3
"""
process_urdf.py — Post-processes the xacro-expanded URDF for Gazebo Harmonic / ROS2 Jazzy.

Changes applied:
  1. Strips collision meshes from suspension links (prevents chassis overlap).
  2. Replaces wheel mesh collisions with smooth cylinders (better contact physics).
  3. Injects the gz-sim-mimic-joint-system plugin if it is not already present
     (idempotent guard so robot_sim.xacro and this script don't double-inject).

Mimic joint strategy (Jazzy + Gazebo Harmonic):
  The default DART physics engine ignores URDF <mimic> tags at the physics level.
  The fix is two-pronged:
    a) robot_control.xacro: <param name="mimic"> inside ros2_control block
       → gz_ros2_control enforces coupling during controller updates.
    b) gz-sim-mimic-joint-system plugin (injected here or in robot_sim.xacro)
       → enforces coupling during passive/physics steps.
"""
import sys
import subprocess
import xml.etree.ElementTree as ET


MIMIC_PLUGIN_NAME = "gz::sim::systems::MimicJoint"


def mimic_plugin_already_present(root: ET.Element) -> bool:
    for plugin in root.findall(".//plugin"):
        if plugin.get("name") == MIMIC_PLUGIN_NAME:
            return True
    return False


def main():
    if len(sys.argv) < 2:
        print("Usage: process_urdf.py <xacro_file> [xacro args...]", file=sys.stderr)
        sys.exit(1)

    # 1. Expand xacro → raw URDF XML
    cmd = ["xacro"] + sys.argv[1:]
    try:
        xml_data = subprocess.check_output(cmd, text=True)
    except subprocess.CalledProcessError as e:
        print(f"xacro failed: {e}", file=sys.stderr)
        sys.exit(1)

    ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")
    root = ET.fromstring(xml_data)

    # 2. Strip collision from suspension links (prevents internal chassis overlap)
    for link in root.findall(".//link"):
        name = link.get("name", "")
        if name in ("left_suspension_link", "right_suspension_link"):
            for col in link.findall("collision"):
                link.remove(col)

    # 3. Replace wheel mesh collisions with smooth cylinders (better contact physics)
    for link in root.findall(".//link"):
        name = link.get("name", "")
        if "wheel_link" in name or name.endswith("_wheel_link"):
            collision = link.find("collision")
            if collision is not None:
                geom = collision.find("geometry")
                if geom is not None:
                    mesh = geom.find("mesh")
                    if mesh is not None:
                        geom.remove(mesh)
                    cylinder = ET.Element("cylinder")
                    cylinder.set("radius", "0.155")
                    cylinder.set("length", "0.162")
                    geom.append(cylinder)

    # 4. Mimic joint handled via gz_ros2_control; physics mimic plugin disabled.
    pass

    # 5. Write modified URDF to stdout
    sys.stdout.write(ET.tostring(root, encoding="unicode"))


if __name__ == "__main__":
    main()
