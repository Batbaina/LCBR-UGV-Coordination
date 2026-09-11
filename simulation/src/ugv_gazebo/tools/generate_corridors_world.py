#!/usr/bin/env python3

from pathlib import Path
import math
import yaml


PACKAGE = Path(__file__).resolve().parents[1]

SCENARIO = PACKAGE / "scenarios" / "paper_corridors_6ugv.yaml"
OUTPUT = PACKAGE / "worlds" / "paper_corridors_6ugv.sdf"

OBS_RADIUS = 0.8
OBS_HEIGHT = 1.5


def material(r, g, b, a=1.0):
    return f"""
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>"""


def cylinder_obstacle(i, x, y):
    return f"""
    <model name="obstacle_{i:03d}">
      <static>true</static>
      <pose>{x} {y} {OBS_HEIGHT/2.0} 0 0 0</pose>

      <link name="link">

        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>{OBS_RADIUS}</radius>
              <length>{OBS_HEIGHT}</length>
            </cylinder>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>{OBS_RADIUS}</radius>
              <length>{OBS_HEIGHT}</length>
            </cylinder>
          </geometry>
{material(0.25, 0.25, 0.25)}
        </visual>

      </link>
    </model>
"""


def marker(name, x, y, r, g, b):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.025 0 0 0</pose>

      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.55</radius>
              <length>0.05</length>
            </cylinder>
          </geometry>
{material(r, g, b)}
        </visual>
      </link>
    </model>
"""


def boundary(name, x, y, sx, sy):
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.03 0 0 0</pose>

      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>{sx} {sy} 0.06</size>
            </box>
          </geometry>
{material(0.85, 0.10, 0.10)}
        </visual>
      </link>
    </model>
"""


def main():

    with SCENARIO.open() as f:
        data = yaml.safe_load(f)

    width, height = data["map"]["dimensions"]

    cx = width / 2.0
    cy = height / 2.0

    robots = data["robots"]
    pois = data["pois"]
    obstacles = data["map"]["obstacles"]

    out = []

    out.append("""<?xml version="1.0"?>
<sdf version="1.10">

  <world name="paper_corridors_6ugv">

    <physics name="physics" type="ignored">
      <max_step_size>0.02</max_step_size>
      <real_time_factor>80.0</real_time_factor>
    </physics>

    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics"/>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands"/>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>27.5 27.5 60 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
""")

    # -------------------------------------------------------
    # Exact planner workspace
    # -------------------------------------------------------

    out.append(f"""
    <model name="planner_workspace">
      <static>true</static>

      <pose>{cx} {cy} -0.05 0 0 0</pose>

      <link name="link">

        <collision name="collision">
          <geometry>
            <box>
              <size>{width + 20} {height + 20} 0.10</size>
            </box>
          </geometry>
        </collision>

        <visual name="visual">
          <geometry>
            <box>
              <size>{width} {height} 0.10</size>
            </box>
          </geometry>
{material(0.65, 0.65, 0.65)}
        </visual>

      </link>
    </model>
""")

    # -------------------------------------------------------
    # Visual boundaries of the PHYSICAL Gazebo support.
    #
    # Planner workspace remains:
    #   x in [0,width], y in [0,height]
    #
    # Physical support has a 10 m safety margin on every side:
    #   x in [-10,width+10]
    #   y in [-10,height+10]
    # -------------------------------------------------------

    physical_margin = 10.0

    physical_width = width + 2.0 * physical_margin
    physical_height = height + 2.0 * physical_margin

    x_min_phys = -physical_margin
    x_max_phys = width + physical_margin

    y_min_phys = -physical_margin
    y_max_phys = height + physical_margin

    out.append(boundary(
        "boundary_south",
        cx, y_min_phys,
        physical_width, 0.12
    ))

    out.append(boundary(
        "boundary_north",
        cx, y_max_phys,
        physical_width, 0.12
    ))

    out.append(boundary(
        "boundary_west",
        x_min_phys, cy,
        0.12, physical_height
    ))

    out.append(boundary(
        "boundary_east",
        x_max_phys, cy,
        0.12, physical_height
    ))

    # -------------------------------------------------------
    # Static obstacles
    # -------------------------------------------------------

    for i, obs in enumerate(obstacles):
        x = float(obs[0])
        y = float(obs[1])

        out.append(
            cylinder_obstacle(i, x, y)
        )

    # -------------------------------------------------------
    # Starts
    # -------------------------------------------------------

    for i, robot in enumerate(robots):
        x, y, yaw = robot["start"]

        out.append(
            marker(
                f"start_{i}",
                x, y,
                0.10, 0.35, 0.95
            )
        )

    # -------------------------------------------------------
    # Goals / POIs
    # -------------------------------------------------------

    for i, poi in enumerate(pois):
        x, y, yaw = poi["goal"]

        out.append(
            marker(
                f"goal_{i}",
                x, y,
                0.10, 0.85, 0.25
            )
        )

    # -------------------------------------------------------
    # Six UGVs
    #
    # Planner:
    #   agent0 -> ugv_1
    #   agent1 -> ugv_2
    #   ...
    #   agent5 -> ugv_6
    #
    # Planner yaw and Gazebo yaw use opposite signs.
    # -------------------------------------------------------

    for i, robot in enumerate(robots):

        sx, sy, syaw = robot["start"]

        gazebo_yaw = -float(syaw)

        model_id = i + 1

        out.append(f"""
    <include>
      <uri>model://ugv_{model_id}</uri>
      <name>ugv_{model_id}</name>
      <pose>{sx} {sy} 0.05 0 0 {gazebo_yaw}</pose>
    </include>
""")

    out.append("""
  </world>

</sdf>
""")

    OUTPUT.write_text("".join(out))

    print("=" * 72)
    print("GAZEBO WORLD GENERATED FROM PLANNER SCENARIO")
    print("=" * 72)
    print(f"Scenario   : {SCENARIO}")
    print(f"Output     : {OUTPUT}")
    print(f"Workspace  : {width} x {height} m")
    print(f"Obstacles  : {len(obstacles)}")
    print(f"Starts     : {len(robots)}")
    print(f"Goals      : {len(pois)}")
    print(f"obsRadius  : {OBS_RADIUS} m")
    print()
    print("UGV spawns:")

    for i, robot in enumerate(robots):
        sx, sy, syaw = robot["start"]
        gazebo_yaw = -float(syaw)

        print(
            f"  ugv_{i + 1}: "
            f"({sx}, {sy}, yaw_G={gazebo_yaw:.4f})"
        )


if __name__ == "__main__":
    main()
