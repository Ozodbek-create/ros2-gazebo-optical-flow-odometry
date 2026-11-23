#!/usr/bin/env python3

import argparse
import random
from pathlib import Path

random.seed(42)

world = """
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="default">

    <gravity>0 0 -9.81</gravity>

    <plugin filename="gz-sim-physics-system"
            name="gz::sim::systems::Physics"/>

    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>

    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>

    <plugin filename="gz-sim-sensors-system"
            name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.3 0.2 -1</direction>
      <cast_shadows>false</cast_shadows>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">

        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
        </collision>

        <visual name="floor">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>20 20</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
"""

# Create a patterned test area near the robot.
# The markings are visual only, so they do not
# interfere with the wheels or floor physics.

for i in range(28):
    for j in range(28):
        x = (i - 13.5) * 0.035
        y = (j - 13.5) * 0.035

        x += random.uniform(-0.009, 0.009)
        y += random.uniform(-0.009, 0.009)

        width = random.uniform(0.008, 0.022)
        length = random.uniform(0.008, 0.022)
        angle = random.uniform(0, 3.14159)
        shade = random.choice([0.08, 0.25, 0.85, 0.95])

        world += f"""
        <visual name="mark_{i}_{j}">
          <pose>{x} {y} 0.001 0 0 {angle}</pose>
          <geometry>
            <box>
              <size>{width} {length} 0.001</size>
            </box>
          </geometry>
          <material>
            <ambient>{shade} {shade} {shade} 1</ambient>
            <diffuse>{shade} {shade} {shade} 1</diffuse>
          </material>
        </visual>
"""

world += """
      </link>
    </model>
  </world>
</sdf>
"""

parser = argparse.ArgumentParser(description="Generate the seeded optical-flow world")
parser.add_argument("--output", default="worlds/optical_flow.sdf")
path = Path(parser.parse_args().output)
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(world.strip())

print(f"World generated: {path.resolve()}")
