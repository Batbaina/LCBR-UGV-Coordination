#!/usr/bin/env python3

import math
from pathlib import Path
import yaml


POINTS_PER_PRIMITIVE = 20
R = 3.0


def wrap_2pi(a):
    return a % (2.0 * math.pi)


def direction_from_action(action):
    if action in (0, 1, 2):
        return "forward"
    if action in (3, 4, 5):
        return "reverse"
    if action == 6:
        return "wait"
    return "unknown"


def signed_yaw_delta(yaw0, yaw1, action):

    if action in (0, 3, 6):
        return 0.0

    raw = (yaw1 - yaw0) % (2.0 * math.pi)

    if action in (1, 5):
        return raw

    # actions 2 and 4 have negative planner dyaw
    if raw > 0.0:
        raw -= 2.0 * math.pi

    return raw


def exact_primitive_points(a, b, n=POINTS_PER_PRIMITIVE):
    """
    Reconstruct a dense reference from the SHA* primitive.

    The planner convention uses:
        x' = x + dx*cos(-yaw) - dy*sin(-yaw)
        y' = y + dx*sin(-yaw) + dy*cos(-yaw)

    Turning primitives are sampled as circular arcs with R=3 m.
    Straight primitives are sampled linearly.
    WAIT keeps the pose fixed.
    """

    action = int(a["action"])
    direction = direction_from_action(action)

    x0 = float(a["x"])
    y0 = float(a["y"])
    yaw0 = float(a["yaw"])
    t0 = float(a["t"])

    x1 = float(b["x"])
    y1 = float(b["y"])
    yaw1 = float(b["yaw"])
    t1 = float(b["t"])

    # --------------------------------------------------
    # WAIT
    # --------------------------------------------------
    if action == 6:
        return [
            {
                "x": x0,
                "y": y0,
                "yaw": wrap_2pi(yaw0),
                "t": t0 + (t1 - t0) * j / n,
                "action": action,
                "direction": direction,
            }
            for j in range(n)
        ]

    # --------------------------------------------------
    # STRAIGHT
    # --------------------------------------------------
    if action in (0, 3):

        points = []

        for j in range(n):
            alpha = j / n

            points.append({
                "x": x0 + alpha * (x1 - x0),
                "y": y0 + alpha * (y1 - y0),
                "yaw": wrap_2pi(yaw0),
                "t": t0 + alpha * (t1 - t0),
                "action": action,
                "direction": direction,
            })

        return points

    # --------------------------------------------------
    # TURNING PRIMITIVE
    # --------------------------------------------------

    delta = signed_yaw_delta(
        yaw0,
        yaw1,
        action
    )

    points = []

    # We reconstruct the primitive by applying the same
    # local circular-motion equations at fractional angle.
    #
    # Forward:
    #   local dx = R sin(|dtheta|)
    #
    # Reverse:
    #   local dx has the opposite sign.
    #
    # The sign of local dy follows the action convention.

    forward = action < 3

    for j in range(n):

        alpha = j / n
        dtheta = alpha * delta

        ad = abs(dtheta)

        local_dx = R * math.sin(ad)

        if not forward:
            local_dx = -local_dx

        # Match Constants::dy[] convention.
        if action in (1, 4):
            local_dy = -R * (1.0 - math.cos(ad))
        else:
            local_dy = +R * (1.0 - math.cos(ad))

        x = (
            x0
            + local_dx * math.cos(-yaw0)
            - local_dy * math.sin(-yaw0)
        )

        y = (
            y0
            + local_dx * math.sin(-yaw0)
            + local_dy * math.cos(-yaw0)
        )

        yaw = wrap_2pi(
            yaw0 + dtheta
        )

        t = t0 + alpha * (t1 - t0)

        points.append({
            "x": x,
            "y": y,
            "yaw": yaw,
            "t": t,
            "action": action,
            "direction": direction,
        })

    return points


def build_dense_reference(states):

    dense = []

    for k in range(len(states) - 1):

        if "action" not in states[k]:
            raise RuntimeError(
                f"State {k} has no action."
            )

        dense.extend(
            exact_primitive_points(
                states[k],
                states[k + 1]
            )
        )

    goal = states[-1]

    dense.append({
        "x": float(goal["x"]),
        "y": float(goal["y"]),
        "yaw": wrap_2pi(float(goal["yaw"])),
        "t": float(goal["t"]),
        "action": -1,
        "direction": "goal",
    })

    return dense


def endpoint_audit(states):

    print()
    print("Primitive endpoint audit:")
    print()

    max_position_error = 0.0
    max_yaw_error = 0.0

    for k in range(len(states) - 1):

        a = states[k]
        b = states[k + 1]

        action = int(a["action"])

        # Generate n+1 so final sample corresponds to alpha=1.
        samples = exact_primitive_points(
            a, b, POINTS_PER_PRIMITIVE
        )

        # Function normally excludes alpha=1.
        # Reconstruct endpoint separately by using a temporary
        # interpolation with one extra fraction.
        if action == 6:
            xe = float(a["x"])
            ye = float(a["y"])
            yawe = float(a["yaw"])

        elif action in (0, 3):
            xe = float(b["x"])
            ye = float(b["y"])
            yawe = float(a["yaw"])

        else:
            yaw0 = float(a["yaw"])
            yaw1 = float(b["yaw"])

            delta = signed_yaw_delta(
                yaw0, yaw1, action
            )

            ad = abs(delta)

            local_dx = R * math.sin(ad)

            if action >= 3:
                local_dx = -local_dx

            if action in (1, 4):
                local_dy = -R * (1.0 - math.cos(ad))
            else:
                local_dy = +R * (1.0 - math.cos(ad))

            xe = (
                float(a["x"])
                + local_dx * math.cos(-yaw0)
                - local_dy * math.sin(-yaw0)
            )

            ye = (
                float(a["y"])
                + local_dx * math.sin(-yaw0)
                + local_dy * math.cos(-yaw0)
            )

            yawe = wrap_2pi(
                yaw0 + delta
            )

        target_x = float(b["x"])
        target_y = float(b["y"])
        target_yaw = wrap_2pi(float(b["yaw"]))

        ep = math.hypot(
            xe - target_x,
            ye - target_y
        )

        eyaw = abs(
            math.atan2(
                math.sin(yawe - target_yaw),
                math.cos(yawe - target_yaw)
            )
        )

        max_position_error = max(
            max_position_error, ep
        )

        max_yaw_error = max(
            max_yaw_error, eyaw
        )

        if k < 10:
            print(
                f"{k:2d}: "
                f"a={action} "
                f"ep={ep:.6f} m "
                f"eyaw={math.degrees(eyaw):.6f} deg"
            )

    print()
    print(
        f"Max endpoint position error: "
        f"{max_position_error:.9f} m"
    )

    print(
        f"Max endpoint yaw error     : "
        f"{math.degrees(max_yaw_error):.9f} deg"
    )



def audit_agent(states):
    """
    Return quantitative audit information without changing the
    validated primitive reconstruction.
    """

    max_position_error = 0.0
    max_yaw_error = 0.0

    action_counts = {
        "forward": 0,
        "reverse": 0,
        "wait": 0,
    }

    wait_intervals = []

    for k in range(len(states) - 1):

        a = states[k]
        b = states[k + 1]

        action = int(a["action"])
        direction = direction_from_action(action)

        if direction in action_counts:
            action_counts[direction] += 1

        if action == 6:
            wait_intervals.append(
                (
                    float(a["t"]),
                    float(b["t"])
                )
            )

        # ----------------------------------------------
        # Reconstruct exact endpoint using the same
        # equations already validated for agent0.
        # ----------------------------------------------

        if action == 6:

            xe = float(a["x"])
            ye = float(a["y"])
            yawe = wrap_2pi(float(a["yaw"]))

        elif action in (0, 3):

            xe = float(b["x"])
            ye = float(b["y"])
            yawe = wrap_2pi(float(a["yaw"]))

        else:

            yaw0 = float(a["yaw"])
            yaw1 = float(b["yaw"])

            delta = signed_yaw_delta(
                yaw0,
                yaw1,
                action
            )

            ad = abs(delta)

            local_dx = R * math.sin(ad)

            if action >= 3:
                local_dx = -local_dx

            if action in (1, 4):
                local_dy = -R * (
                    1.0 - math.cos(ad)
                )
            else:
                local_dy = +R * (
                    1.0 - math.cos(ad)
                )

            xe = (
                float(a["x"])
                + local_dx * math.cos(-yaw0)
                - local_dy * math.sin(-yaw0)
            )

            ye = (
                float(a["y"])
                + local_dx * math.sin(-yaw0)
                + local_dy * math.cos(-yaw0)
            )

            yawe = wrap_2pi(
                yaw0 + delta
            )

        target_x = float(b["x"])
        target_y = float(b["y"])
        target_yaw = wrap_2pi(
            float(b["yaw"])
        )

        ep = math.hypot(
            xe - target_x,
            ye - target_y
        )

        eyaw = abs(
            math.atan2(
                math.sin(
                    yawe - target_yaw
                ),
                math.cos(
                    yawe - target_yaw
                )
            )
        )

        max_position_error = max(
            max_position_error,
            ep
        )

        max_yaw_error = max(
            max_yaw_error,
            eyaw
        )

    return {
        "max_position_error":
            max_position_error,

        "max_yaw_error":
            max_yaw_error,

        "forward":
            action_counts["forward"],

        "reverse":
            action_counts["reverse"],

        "wait":
            action_counts["wait"],

        "wait_intervals":
            wait_intervals,
    }


def main():

    package_dir = (
        Path(__file__).resolve().parents[1]
    )

    source = (
        package_dir
        / "trajectories"
        / "final_lcbr.yaml"
    )

    output_dir = (
        package_dir
        / "trajectories"
        / "dense"
    )

    output_dir.mkdir(
        parents=True,
        exist_ok=True
    )

    with source.open("r") as f:
        data = yaml.safe_load(f)

    schedule = data["schedule"]

    # Sort numerically:
    # agent0, agent1, ..., agent5
    agents = sorted(
        schedule.keys(),
        key=lambda name: int(
            name.replace("agent", "")
        )
    )

    print("=" * 78)
    print("MULTI-UGV EXACT SHA* DENSE REFERENCES")
    print("=" * 78)

    print(
        f"Source               : {source}"
    )

    print(
        f"Points / primitive   : "
        f"{POINTS_PER_PRIMITIVE}"
    )

    print(
        f"Agents               : {len(agents)}"
    )

    print()

    global_max_ep = 0.0
    global_max_eyaw = 0.0

    total_forward = 0
    total_reverse = 0
    total_wait = 0

    summaries = []

    for agent in agents:

        states = schedule[agent]

        if len(states) < 1:
            raise RuntimeError(
                f"{agent} has no states"
            )

        # Validate state/action structure.
        for k in range(len(states) - 1):

            if "action" not in states[k]:
                raise RuntimeError(
                    f"{agent}: state {k} "
                    f"has no action"
                )

            action = int(
                states[k]["action"]
            )

            if action not in (
                0, 1, 2, 3, 4, 5, 6
            ):
                raise RuntimeError(
                    f"{agent}: invalid action "
                    f"{action} at state {k}"
                )

        dense = build_dense_reference(
            states
        )

        audit = audit_agent(
            states
        )

        output = (
            output_dir
            / f"{agent}_dense.yaml"
        )

        result = {
            "source":
                "final_lcbr.yaml",

            "agent":
                agent,

            "points_per_primitive":
                POINTS_PER_PRIMITIVE,

            "num_original_states":
                len(states),

            "num_original_transitions":
                max(0, len(states) - 1),

            "num_dense_points":
                len(dense),

            "start_time":
                float(states[0]["t"]),

            "goal_time":
                float(states[-1]["t"]),

            "motion_summary": {
                "forward":
                    audit["forward"],

                "reverse":
                    audit["reverse"],

                "wait":
                    audit["wait"],
            },

            "wait_intervals": [
                {
                    "start": t0,
                    "end": t1,
                }
                for t0, t1
                in audit["wait_intervals"]
            ],

            "trajectory":
                dense,
        }

        with output.open("w") as f:

            yaml.safe_dump(
                result,
                f,
                sort_keys=False
            )

        global_max_ep = max(
            global_max_ep,
            audit["max_position_error"]
        )

        global_max_eyaw = max(
            global_max_eyaw,
            audit["max_yaw_error"]
        )

        total_forward += audit["forward"]
        total_reverse += audit["reverse"]
        total_wait += audit["wait"]

        summaries.append(
            (
                agent,
                len(states),
                len(states) - 1,
                len(dense),
                audit
            )
        )

    # --------------------------------------------------
    # Console summary
    # --------------------------------------------------

    for (
        agent,
        nstates,
        ntrans,
        ndense,
        audit
    ) in summaries:

        print(
            f"{agent:7s} | "
            f"states={nstates:3d} | "
            f"trans={ntrans:3d} | "
            f"dense={ndense:4d} | "
            f"F={audit['forward']:2d} "
            f"R={audit['reverse']:2d} "
            f"W={audit['wait']:2d} | "
            f"ep_max="
            f"{audit['max_position_error']:.6f} m | "
            f"eyaw_max="
            f"{math.degrees(audit['max_yaw_error']):.6f} deg"
        )

        if audit["wait_intervals"]:

            waits = ", ".join(
                f"[{t0:g},{t1:g}]"
                for t0, t1
                in audit["wait_intervals"]
            )

            print(
                f"          WAIT: {waits}"
            )

    print()
    print("-" * 78)

    print(
        f"Total transitions     : "
        f"{total_forward + total_reverse + total_wait}"
    )

    print(
        f"Forward transitions   : "
        f"{total_forward}"
    )

    print(
        f"Reverse transitions   : "
        f"{total_reverse}"
    )

    print(
        f"WAIT transitions      : "
        f"{total_wait}"
    )

    print()

    print(
        f"GLOBAL max endpoint position error : "
        f"{global_max_ep:.9f} m"
    )

    print(
        f"GLOBAL max endpoint yaw error      : "
        f"{math.degrees(global_max_eyaw):.9f} deg"
    )

    print()

    print(
        f"Output directory      : "
        f"{output_dir}"
    )

    print("=" * 78)


if __name__ == "__main__":
    main()
