#!/usr/bin/env python3
"""
generate_instance.py -- generates robots/POIs/map YAML instances with a
choice of obstacle patterns, for testing the LCBR pipeline (run_full_pipeline.py)
under varied conditions instead of a single fixed wall-with-a-gap layout.

Patterns:
  empty            no obstacles at all.
  random           scattered circular obstacles at random positions
                    (density controlled by --density, obstacles per 100 m^2).
  wall_single_gap  one horizontal wall across the map with exactly one gap
                    (the "bottleneck" pattern used earlier).
  wall_multi_gap   one horizontal wall with several gaps, so robots have a
                    choice of which gap to use.
  double_wall      two parallel walls (each with its own gap, at different
                    x-positions), forcing an S-shaped detour.
  clusters         several separate round clusters ("rock piles") scattered
                    across the map, rather than a continuous wall.
  maze             a small set of offset wall segments forming simple
                    corridors.

Robots are placed on one side / region and POIs on another, with random
jitter, then both are checked against the generated obstacles (and against
each other) with a safety margin, retrying placement if too close.

Usage:
  python3 generate_instance.py --pattern random --num-robots 4 \\
      --map-size 40 --seed 1 --output instance_random.yaml

  # generate one of every pattern in one go:
  python3 generate_instance.py --all-patterns --num-robots 4 \\
      --map-size 40 --seed 1 --output-dir instances/
"""
import argparse
import math
import random
import subprocess
import tempfile
from pathlib import Path

import yaml

PATTERNS = ["empty", "random", "wall_single_gap", "wall_multi_gap",
           "double_wall", "clusters", "maze", "crop_rows", "scattered_and_walls"]
START_MODES = ["scattered", "edges", "one_edge", "grouped_corner"]
EDGE_SIDES = ["left", "right", "top", "bottom"]
CORNERS = ["bottom_left", "bottom_right", "top_left", "top_right"]

OBS_RADIUS = 0.8   # matches config.yaml's obsRadius
CAR_CLEARANCE = 2.2  # minimum distance kept between a robot/POI and any obstacle center


def wall_row(y, dimx, gaps, spacing=1.5):
    """One horizontal row of circular obstacles at height y, x in [0,dimx],
    skipping any x that falls inside one of the given (lo,hi) gap intervals."""
    obstacles = []
    x = 0.0
    while x <= dimx:
        if not any(lo <= x <= hi for lo, hi in gaps):
            obstacles.append([round(x, 2), y])
        x += spacing
    return obstacles


def gen_obstacles(pattern, dimx, dimy, rng):
    if pattern == "empty":
        return []

    if pattern == "random":
        area = dimx * dimy
        density_per_100 = 3.5
        n = max(3, int(area / 100 * density_per_100))
        obstacles = []
        min_obstacle_spacing = 2.2  # prevents obstacles clustering into an
                                    # accidental wall / enclosure a 3x2m
                                    # car could get trapped behind
        tries = 0
        while len(obstacles) < n and tries < n * 40:
            tries += 1
            cand = [round(rng.uniform(3, dimx - 3), 2),
                   round(rng.uniform(3, dimy - 3), 2)]
            if all(math.hypot(cand[0] - o[0], cand[1] - o[1]) >= min_obstacle_spacing
                  for o in obstacles):
                obstacles.append(cand)
        return obstacles

    if pattern == "wall_single_gap":
        mid = dimx / 2
        gap = (mid - 2.5, mid + 2.5)
        return wall_row(dimy / 2, dimx, [gap])

    if pattern == "wall_multi_gap":
        gaps = [(dimx * 0.2 - 2, dimx * 0.2 + 2),
               (dimx * 0.55 - 2, dimx * 0.55 + 2),
               (dimx * 0.85 - 2, dimx * 0.85 + 2)]
        return wall_row(dimy / 2, dimx, gaps)

    if pattern == "double_wall":
        gap1 = (dimx * 0.25 - 2.5, dimx * 0.25 + 2.5)
        gap2 = (dimx * 0.7 - 2.5, dimx * 0.7 + 2.5)
        wall1 = wall_row(dimy * 0.35, dimx, [gap1])
        wall2 = wall_row(dimy * 0.65, dimx, [gap2])
        return wall1 + wall2

    if pattern == "clusters":
        obstacles = []
        n_clusters = max(3, int((dimx * dimy) / 220))
        for _ in range(n_clusters):
            cx = rng.uniform(6, dimx - 6)
            cy = rng.uniform(6, dimy - 6)
            n_pts = rng.randint(4, 9)
            radius = rng.uniform(1.5, 3.5)
            for _ in range(n_pts):
                ang = rng.uniform(0, 2 * math.pi)
                r = rng.uniform(0, radius)
                obstacles.append([round(cx + r * math.cos(ang), 2),
                                  round(cy + r * math.sin(ang), 2)])
        return obstacles

    if pattern == "maze":
        obstacles = []
        # three offset horizontal segments forming a zig-zag corridor
        obstacles += wall_row(dimy * 0.25, dimx * 0.7, [(dimx * 0.55, dimx * 0.7)])
        obstacles += wall_row(dimy * 0.5, dimx, [(dimx * 0.15, dimx * 0.30)],
                              spacing=1.5)
        obstacles += wall_row(dimy * 0.75, dimx, [(dimx * 0.55, dimx * 0.7)])
        return obstacles

    if pattern == "crop_rows":
        # Agricultural field: several parallel short obstacle rows (crop
        # rows / hedges), running vertically, spaced widely enough apart to
        # form navigable corridors between them -- mimicking a row-crop
        # field a ground robot must weave through, rather than a single
        # wall to pass through once. Rows don't span the full field height,
        # leaving open access strips at the top and bottom.
        obstacles = []
        n_rows = max(3, int(dimx / 8))
        row_spacing = dimx / (n_rows + 1)
        row_height = dimy * 0.7
        y0 = (dimy - row_height) / 2
        for k in range(1, n_rows + 1):
            x = k * row_spacing
            y = y0
            while y <= y0 + row_height:
                obstacles.append([round(x, 2), round(y, 2)])
                y += 1.6
        return obstacles

    if pattern == "scattered_and_walls":
        # Combination requested for a mixed-difficulty scenario: a light
        # scatter of standalone obstacles (rocks/equipment) PLUS two
        # parallel walls with gaps (like double_wall), so robots must both
        # dodge individual obstacles and funnel through two bottlenecks.
        obstacles = []
        area = dimx * dimy
        n = max(3, int(area / 100 * 2.0))  # lighter density than "random"
                                            # alone, to leave the walls
                                            # clearly readable
        min_obstacle_spacing = 2.2
        tries = 0
        while len(obstacles) < n and tries < n * 40:
            tries += 1
            cand = [round(rng.uniform(3, dimx - 3), 2),
                   round(rng.uniform(3, dimy - 3), 2)]
            if all(math.hypot(cand[0] - o[0], cand[1] - o[1]) >= min_obstacle_spacing
                  for o in obstacles):
                obstacles.append(cand)
        gap1 = (dimx * 0.25 - 2.5, dimx * 0.25 + 2.5)
        gap2 = (dimx * 0.7 - 2.5, dimx * 0.7 + 2.5)
        obstacles += wall_row(dimy * 0.35, dimx, [gap1])
        obstacles += wall_row(dimy * 0.65, dimx, [gap2])
        return obstacles

    raise ValueError(f"unknown pattern {pattern}")


def reserve_center_clear_zone(obstacles, dimx, dimy, radius=6.0):
    """Remove any obstacle within `radius` of the map centre. The centre is
    used as a fixed, always-reachable reference point by the SHA*-based
    validator below, so it must never be blocked."""
    cx, cy = dimx / 2, dimy / 2
    return [o for o in obstacles if math.hypot(o[0] - cx, o[1] - cy) > radius]


class ReachabilityValidator:
    """Validates candidate points with the REAL planner (SHA*, via the
    CL-CBS binary) instead of a geometric heuristic. A heuristic clearance
    check (has_escape_route) can pass a point that is still impossible to
    reach with the exact required final heading -- observed directly while
    building this generator: every robot failed, and hung for a long time,
    trying to reach one specific POI that looked fine geometrically. This
    validator catches that by actually running a short SHA* query from a
    fixed, always-clear reference point (the map centre) to the candidate,
    with a timeout; a point that fails or times out is rejected outright.
    """

    def __init__(self, build_dir, dimx, dimy, timeout=5.0, enabled=True,
                vehicle_config=None):
        self.enabled = enabled and build_dir is not None
        self.timeout = timeout
        self.dimx, self.dimy = dimx, dimy
        self.reference = (dimx / 2, dimy / 2, 0.0)
        self.binary = None
        # Path to config/vehicle_config.yaml -- required by ugv_coordination
        # (unlike the original CL-CBS binary, --vehicle-config has no
        # working default when invoked from an arbitrary temp directory,
        # since its fallback default is a relative "../config/..." path
        # that only resolves correctly when run from build/).
        self.vehicle_config = vehicle_config
        if self.enabled:
            candidate = Path(build_dir).resolve() / "ugv_coordination"
            if candidate.exists():
                self.binary = candidate
            else:
                print(f"WARNING: {candidate} not found -- disabling "
                     f"planner-based validation (falling back to the "
                     f"geometric heuristic only). Pass --build-dir "
                     f"pointing at your compiled ugv_coordination binary "
                     f"to enable it.")
                self.enabled = False
            if self.enabled and (not self.vehicle_config or
                                 not Path(self.vehicle_config).exists()):
                print(f"WARNING: --vehicle-config not given or not found "
                     f"({self.vehicle_config}) -- disabling planner-based "
                     f"validation. Pass --vehicle-config pointing at "
                     f"config/vehicle_config.yaml to enable it.")
                self.enabled = False

    def is_reachable(self, x, y, yaw, obstacles):
        if not self.enabled:
            return True
        return self._query(self.reference, (x, y, yaw), obstacles)

    def pair_reachable(self, start_xyz, goal_xyz, obstacles):
        """Like is_reachable, but for an arbitrary (start -> goal) pair,
        not just (map centre -> point). Used for the final full MxQ
        validation pass in generate_instance(): the centre-based check
        alone can pass two points that are each individually reachable
        from the centre, yet whose DIRECT pairwise query is pathologically
        hard or unreachable (different obstacle geometry along that
        specific path) -- observed directly while testing a
        densely-obstacled instance."""
        if not self.enabled:
            return True
        return self._query(start_xyz, goal_xyz, obstacles)

    def _query(self, start_xyz, goal_xyz, obstacles):
        # ugv_coordination auto-detects the "agents:" schema (single agent
        # with both start and goal, matching the original CL-CBS format
        # this validator was designed around) -- no --mode needed, LCBR's
        # default behaves identically to full-horizon CL-CBS for a single
        # agent with no conflicts to repair.
        instance = {
            "agents": [{
                "start": [start_xyz[0], start_xyz[1], start_xyz[2]],
                "name": "agent0",
                "goal": [goal_xyz[0], goal_xyz[1], goal_xyz[2]],
            }],
            "map": {"dimensions": [int(self.dimx), int(self.dimy)],
                    "obstacles": obstacles},
        }
        with tempfile.TemporaryDirectory() as td:
            in_path = Path(td) / "in.yaml"
            out_path = Path(td) / "out.yaml"
            with open(in_path, "w") as f:
                yaml.dump(instance, f)
            try:
                res = subprocess.run(
                    [str(self.binary), "-i", str(in_path), "-o", str(out_path),
                    "--vehicle-config", str(self.vehicle_config),
                    "--timeout", str(max(1, int(self.timeout))),
                    "--log-dir", ""],
                    capture_output=True, text=True, timeout=self.timeout + 2)
                return "Successfully" in res.stdout
            except subprocess.TimeoutExpired:
                return False


def has_escape_route(p, obstacles, dimx, dimy, clear_dist=5.0, n_dirs=12):
    """Reject points that are only 'individually far enough' from every
    obstacle but collectively boxed in by a ring of them (e.g. random
    scatter placing 3-4 obstacles just far enough apart, but arranged
    around a point so no direction has room for a car-sized footprint to
    escape). Checks that at least one of n_dirs compass directions has a
    clear straight line of length clear_dist with no obstacle closer than
    CAR_CLEARANCE to it, and stays inside the map."""
    for k in range(n_dirs):
        ang = 2 * math.pi * k / n_dirs
        ok = True
        for step in (clear_dist * 0.3, clear_dist * 0.6, clear_dist):
            qx = p[0] + step * math.cos(ang)
            qy = p[1] + step * math.sin(ang)
            if not (0 <= qx <= dimx and 0 <= qy <= dimy):
                ok = False
                break
            if not far_enough((qx, qy), obstacles, CAR_CLEARANCE):
                ok = False
                break
        if ok:
            return True
    return False


def sample_corner_group_point(dimx, dimy, obstacles, existing_points, rng,
                              validator, corner, zone_frac=0.42,
                              min_spacing=5.0, max_tries=1500):
    """Sample a start point tightly clustered near one map corner -- e.g.
    a team of robots deployed together at a single gate/entry point,
    rather than spread along a whole border (see sample_edge_point).
    Yaw points roughly towards the field centre so the group heads
    inward, with a wider spacing tolerance (min_spacing, smaller than the
    6.0 used elsewhere) since a tight group is exactly the point here."""
    corner_x = {"bottom_left": 0, "top_left": 0,
               "bottom_right": dimx, "top_right": dimx}[corner]
    corner_y = {"bottom_left": 0, "bottom_right": 0,
               "top_left": dimy, "top_right": dimy}[corner]
    zx, zy = dimx * zone_frac, dimy * zone_frac
    lo_x = max(3.0, corner_x - zx if corner_x > 0 else corner_x)
    hi_x = min(dimx - 3.0, corner_x if corner_x > 0 else corner_x + zx)
    lo_y = max(3.0, corner_y - zy if corner_y > 0 else corner_y)
    hi_y = min(dimy - 3.0, corner_y if corner_y > 0 else corner_y + zy)
    center_x, center_y = dimx / 2, dimy / 2

    for _ in range(max_tries):
        x = rng.uniform(lo_x, hi_x)
        y = rng.uniform(lo_y, hi_y)
        yaw = math.atan2(center_y - y, center_x - x) + rng.uniform(-0.2, 0.2)
        yaw = round(yaw, 4)
        if far_enough((x, y), obstacles, CAR_CLEARANCE) and \
           all(math.hypot(x - q[0], y - q[1]) >= min_spacing for q in existing_points) and \
           has_escape_route((x, y), obstacles, dimx, dimy) and \
           validator.is_reachable(x, y, yaw, obstacles):
            return (x, y), yaw
    raise RuntimeError(
        "could not place a grouped-corner start point after many tries -- "
        "the corner zone is too small/crowded for this many robots; try "
        "a larger --map-size or fewer robots")


def far_enough(p, obstacles, min_dist):
    return all(math.hypot(p[0] - o[0], p[1] - o[1]) >= min_dist for o in obstacles)


def sample_free_point(dimx, dimy, obstacles, existing_points, rng, validator,
                      max_tries=800):
    for _ in range(max_tries):
        p = (rng.uniform(2, dimx - 2), rng.uniform(2, dimy - 2))
        if far_enough(p, obstacles, CAR_CLEARANCE) and \
           all(math.hypot(p[0] - q[0], p[1] - q[1]) >= 6.0 for q in existing_points) and \
           has_escape_route(p, obstacles, dimx, dimy):
            return p
    raise RuntimeError("could not place a free point after many tries -- "
                       "map too crowded, try a larger --map-size, fewer "
                       "robots, or a lower obstacle density")


def sample_edge_point(dimx, dimy, obstacles, existing_points, rng, validator,
                      max_tries=500, forced_edge=None):
    """Sample a start point along one of the four map borders (or a single
    specific one, if forced_edge is given), with a yaw pointing inward --
    e.g. tractors/robots entering a field from its boundary, per the
    'agriculture' scenario: robots start at the edge of the map and drive
    in towards their assigned work point."""
    edge_specs = {
        "left": (lambda: rng.uniform(2, dimy - 2), 0.0),
        "right": (lambda: rng.uniform(2, dimy - 2), math.pi),
        "bottom": (lambda: rng.uniform(2, dimx - 2), math.pi / 2),
        "top": (lambda: rng.uniform(2, dimx - 2), -math.pi / 2),
    }
    choices = [forced_edge] if forced_edge else list(edge_specs.keys())
    for _ in range(max_tries):
        name = rng.choice(choices)
        sampler, base_yaw = edge_specs[name]
        v = sampler()
        margin = 3.5  # keep the car's own body (LF=2, LB=1, half-width~1)
                      # fully inside the map even right at the border
        if name == "left":
            p = (margin, v)
        elif name == "right":
            p = (dimx - margin, v)
        elif name == "bottom":
            p = (v, margin)
        else:  # top
            p = (v, dimy - margin)
        yaw = base_yaw + rng.uniform(-0.15, 0.15)
        if far_enough(p, obstacles, CAR_CLEARANCE) and \
           all(math.hypot(p[0] - q[0], p[1] - q[1]) >= 6.0 for q in existing_points) and \
           has_escape_route(p, obstacles, dimx, dimy) and \
           validator.is_reachable(p[0], p[1], round(yaw, 4), obstacles):
            return p, round(yaw, 4)
    raise RuntimeError("could not place an edge start point after many tries -- "
                       "try a larger --map-size, fewer robots, or a "
                       "different --edge")


def generate_instance(pattern, dimx, dimy, num_robots, rng, start_mode="scattered",
                      build_dir=None, validate=True, validate_timeout=5.0,
                      edge_side=None, corner=None, vehicle_config=None):
    obstacles = gen_obstacles(pattern, dimx, dimy, rng)
    obstacles = reserve_center_clear_zone(obstacles, dimx, dimy)
    validator = ReachabilityValidator(build_dir, dimx, dimy,
                                      timeout=validate_timeout, enabled=validate,
                                      vehicle_config=vehicle_config)

    # Yaws are restricted to near-cardinal directions (0, +-pi/2, pi) plus a
    # small jitter. Two reasons: (1) it is more realistic for a field robot
    # / implement, which usually works aligned with rows rather than at an
    # arbitrary angle; (2) an arbitrary goal heading can occasionally force
    # SHA* into a very long sequence of reversing/three-point-turn
    # manoeuvres to match it exactly in a cluttered field, which can make
    # the search pathologically slow (observed directly while testing this
    # generator) -- cardinal-ish headings keep queries tractable, and the
    # validator below catches whatever slips through anyway.
    def easy_yaw():
        base = rng.choice([0.0, math.pi / 2, math.pi, -math.pi / 2])
        return round(base + rng.uniform(-0.15, 0.15), 4)

    def sample_scattered_validated(placed, max_attempts=40):
        for _ in range(max_attempts):
            x, y = sample_free_point(dimx, dimy, obstacles, placed, rng, validator)
            yaw = easy_yaw()
            if validator.is_reachable(x, y, yaw, obstacles):
                return x, y, yaw
        raise RuntimeError("could not find a planner-validated point after "
                          "many attempts -- try a lower obstacle density")

    # For "one_edge", every robot starts along the SAME single border
    # (chosen once, either from --edge or picked at random here) --
    # e.g. tractors all entering the field from the same gate.
    forced_edge = None
    if start_mode == "one_edge":
        forced_edge = edge_side or rng.choice(EDGE_SIDES)

    # For "grouped_corner", every robot starts tightly clustered near the
    # SAME single corner (chosen once, either from --corner or picked at
    # random here) -- e.g. a team deployed together at one gate.
    chosen_corner = None
    if start_mode == "grouped_corner":
        chosen_corner = corner or rng.choice(CORNERS)

    placed = []
    robots = []
    for _ in range(num_robots):
        if start_mode in ("edges", "one_edge"):
            (x, y), yaw = sample_edge_point(dimx, dimy, obstacles, placed, rng,
                                            validator, forced_edge=forced_edge)
        elif start_mode == "grouped_corner":
            (x, y), yaw = sample_corner_group_point(dimx, dimy, obstacles, placed,
                                                     rng, validator, chosen_corner)
        else:
            x, y, yaw = sample_scattered_validated(placed)
        robots.append({"start": [round(x, 2), round(y, 2), yaw]})
        placed.append((x, y))

    # Goals (POIs) are always scattered inside the field, never on the
    # border -- e.g. inspection/work points a field robot must reach,
    # regardless of where it entered the field from.
    pois = []
    for _ in range(num_robots):
        x, y, yaw = sample_scattered_validated(placed)
        pois.append({"goal": [round(x, 2), round(y, 2), yaw]})
        placed.append((x, y))

    # Final safety net: validate every one of the M x Q robot-POI pairs
    # DIRECTLY (not just each point's reachability from the map centre).
    # Two points can each be individually reachable from the centre yet
    # still form a pathologically slow or unreachable DIRECT pair -- this
    # is exactly what Module A will query, so it is what must be checked.
    # Observed directly on a densely-obstacled instance during testing:
    # Module A hung with no diagnostic, because this pairwise check was
    # missing. Only run for a modest number of robots (M*Q queries, each
    # up to validate_timeout seconds) to keep generation time bounded.
    if validator.enabled and num_robots <= 8:
        for i, r in enumerate(robots):
            for j, p in enumerate(pois):
                if not validator.pair_reachable(r["start"], p["goal"], obstacles):
                    raise RuntimeError(
                        f"pairwise validation failed: robot{i} -> poi{j} is "
                        f"unreachable or pathologically slow -- regenerate "
                        f"with a different --seed")

    return {
        "robots": robots,
        "pois": pois,
        "map": {"dimensions": [int(dimx), int(dimy)], "obstacles": obstacles},
    }


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pattern", choices=PATTERNS, default="wall_single_gap")
    ap.add_argument("--start-mode", choices=START_MODES, default="scattered",
                    help="'scattered': robots start anywhere in the field. "
                         "'edges': each robot starts along a (possibly "
                         "different) map border, pointing inward. "
                         "'one_edge': ALL robots start along the SAME "
                         "single border -- e.g. tractors all entering the "
                         "field from the same gate -- see --edge.")
    ap.add_argument("--edge", choices=EDGE_SIDES, default=None,
                    help="which border to use with --start-mode one_edge "
                         "(default: picked randomly, but consistent for "
                         "all robots in the instance)")
    ap.add_argument("--corner", choices=CORNERS, default=None,
                    help="which corner to use with --start-mode "
                         "grouped_corner (default: picked randomly, but "
                         "consistent for all robots in the instance)")
    ap.add_argument("--all-patterns", action="store_true",
                    help="generate one instance per pattern instead of a single one")
    ap.add_argument("--num-robots", type=int, default=4)
    ap.add_argument("--map-size", type=float, default=40,
                    help="square map side length in meters")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--output", default="instance.yaml",
                    help="output path (single-pattern mode)")
    ap.add_argument("--output-dir", default="instances",
                    help="output directory (--all-patterns mode)")
    ap.add_argument("--build-dir", default=None,
                    help="folder containing the compiled ugv_coordination binary, "
                         "used to validate every generated start/goal with the "
                         "real planner (recommended). If omitted, only the "
                         "faster geometric heuristic is used, which can "
                         "occasionally let through a point that is "
                         "unreachable or pathologically slow to reach.")
    ap.add_argument("--vehicle-config", default=None,
                    help="path to config/vehicle_config.yaml -- required if "
                         "--build-dir is given (ugv_coordination has no working "
                         "relative default when invoked from a temp directory)")
    ap.add_argument("--no-validate", action="store_true",
                    help="skip planner-based validation even if --build-dir is given")
    ap.add_argument("--validate-timeout", type=float, default=5.0,
                    help="per-point validation timeout, in seconds")
    args = ap.parse_args()
    max_attempts = 5

    def generate_with_retry(pattern, seed):
        for attempt in range(max_attempts):
            try:
                rng = random.Random(seed + attempt * 10007)
                data = generate_instance(pattern, args.map_size, args.map_size,
                                         args.num_robots, rng, args.start_mode,
                                         build_dir=args.build_dir,
                                         validate=not args.no_validate,
                                         validate_timeout=args.validate_timeout,
                                         edge_side=args.edge, corner=args.corner,
                                         vehicle_config=args.vehicle_config)
                if attempt > 0:
                    print(f"  (succeeded on retry {attempt + 1}/{max_attempts}, "
                         f"effective seed {seed + attempt * 10007})")
                return data
            except RuntimeError as e:
                print(f"  attempt {attempt + 1}/{max_attempts} failed: {e}")
        raise RuntimeError(
            f"could not generate a valid '{pattern}' instance after "
            f"{max_attempts} attempts -- try a larger --map-size, fewer "
            f"--num-robots, or a lower obstacle density for this pattern")

    if args.all_patterns:
        out_dir = Path(args.output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        for pattern in PATTERNS:
            data = generate_with_retry(pattern, args.seed)
            out_path = out_dir / f"instance_{pattern}.yaml"
            with open(out_path, "w") as f:
                yaml.dump(data, f, sort_keys=False)
            print(f"[{pattern:16s}] {len(data['map']['obstacles']):3d} obstacles "
                 f"-> {out_path}")
    else:
        data = generate_with_retry(args.pattern, args.seed)
        with open(args.output, "w") as f:
            yaml.dump(data, f, sort_keys=False)
        print(f"[{args.pattern}, start={args.start_mode}] "
             f"{len(data['map']['obstacles'])} obstacles -> {args.output}")


if __name__ == "__main__":
    main()