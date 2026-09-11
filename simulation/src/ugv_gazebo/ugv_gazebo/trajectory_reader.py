#!/usr/bin/env python3

import math
from pathlib import Path
import yaml


EPS_POSITION = 1e-4
EPS_DIRECTION = 1e-6


def classify_motion(a, b):
    """
    Classify the motion from state a to state b as:
      FORWARD, REVERSE, or WAIT.

    Direction is determined by projecting the displacement
    onto the vehicle heading at state a.
    """

    dx = float(b["x"]) - float(a["x"])
    dy = float(b["y"]) - float(a["y"])

    distance = math.hypot(dx, dy)

    if distance < EPS_POSITION:
        return "WAIT", distance, 0.0

    yaw = float(a["yaw"])

    hx = math.cos(yaw)
    hy = math.sin(yaw)

    projection = dx * hx + dy * hy

    if projection > EPS_DIRECTION:
        motion = "FORWARD"
    elif projection < -EPS_DIRECTION:
        motion = "REVERSE"
    else:
        motion = "AMBIGUOUS"

    return motion, distance, projection


def analyze_agent(name, states):

    if not states:
        print(f"{name}: EMPTY")
        return

    counts = {
        "FORWARD": 0,
        "REVERSE": 0,
        "WAIT": 0,
        "AMBIGUOUS": 0,
    }

    total_distance = 0.0
    transitions = []

    for k in range(len(states) - 1):

        a = states[k]
        b = states[k + 1]

        motion, distance, projection = classify_motion(a, b)

        counts[motion] += 1
        total_distance += distance

        transitions.append({
            "k": k,
            "t0": a["t"],
            "t1": b["t"],
            "motion": motion,
            "distance": distance,
            "projection": projection,
        })

    start = states[0]
    goal = states[-1]

    print("=" * 72)
    print(name)
    print("=" * 72)

    print(f"States      : {len(states)}")
    print(f"Transitions : {len(states) - 1}")

    print(
        "Start       : "
        f"({float(start['x']):.3f}, "
        f"{float(start['y']):.3f}, "
        f"yaw={float(start['yaw']):.4f}, "
        f"t={start['t']})"
    )

    print(
        "Goal        : "
        f"({float(goal['x']):.3f}, "
        f"{float(goal['y']):.3f}, "
        f"yaw={float(goal['yaw']):.4f}, "
        f"t={goal['t']})"
    )

    duration = float(goal["t"]) - float(start["t"])

    print(f"Duration    : {duration:.3f}")
    print(f"Path length : {total_distance:.3f}")

    print()
    print("Motion classification:")
    print(f"  FORWARD   : {counts['FORWARD']}")
    print(f"  REVERSE   : {counts['REVERSE']}")
    print(f"  WAIT      : {counts['WAIT']}")
    print(f"  AMBIGUOUS : {counts['AMBIGUOUS']}")

    print()
    print("First transitions:")

    for tr in transitions[:10]:
        print(
            f"  {tr['t0']} -> {tr['t1']}  "
            f"{tr['motion']:9s}  "
            f"ds={tr['distance']:.3f}  "
            f"proj={tr['projection']:+.3f}"
        )

    # Explicitly display waits
    waits = [
        tr for tr in transitions
        if tr["motion"] == "WAIT"
    ]

    if waits:
        print()
        print("Wait intervals:")
        for tr in waits:
            print(
                f"  t={tr['t0']} -> {tr['t1']}"
            )

    print()


def main():

    # Package source directory:
    package_dir = Path(__file__).resolve().parents[1]

    trajectory_file = (
        package_dir
        / "trajectories"
        / "final_lcbr.yaml"
    )

    if not trajectory_file.is_file():
        raise FileNotFoundError(
            f"Trajectory file not found:\n{trajectory_file}"
        )

    print(f"Reading: {trajectory_file}")
    print()

    with trajectory_file.open("r") as f:
        data = yaml.safe_load(f)

    if "schedule" not in data:
        raise KeyError(
            "The YAML file does not contain a 'schedule' section."
        )

    schedule = data["schedule"]

    print(f"Agents found: {len(schedule)}")
    print()

    for name in sorted(schedule.keys()):
        analyze_agent(name, schedule[name])


if __name__ == "__main__":
    main()
