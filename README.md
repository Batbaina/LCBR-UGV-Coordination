# LCBR — Multi-UGV Coordination and Gazebo Validation

**Local Conflict-Based Trajectory Repair (LCBR)** for coordinated motion planning of multiple Ackermann-steered UGVs, with an end-to-end validation framework in **ROS 2 + Gazebo**.

This repository contains two complementary components:

1. **`algorithm/`** — the planning and coordination framework:
   - SHA* nominal trajectory generation,
   - trajectory-cost-based task allocation,
   - CL-CBS baseline,
   - Local Conflict-Based Trajectory Repair (LCBR).

2. **`simulation/`** — the execution and validation framework:
   - exact reconstruction of SHA* motion primitives,
   - multi-UGV temporal execution,
   - Pure Pursuit trajectory tracking,
   - Ackermann vehicle models,
   - ROS 2 / Gazebo simulation.

The complete workflow is:

```text
Scenario
   │
   ▼
SHA* nominal trajectory generation
   │
   ▼
Trajectory-cost-based task allocation
   │
   ▼
Nominal multi-UGV trajectories
   │
   ▼
CL-CBS or LCBR conflict resolution
   │
   ▼
final_lcbr.yaml
   │
   ▼
Exact SHA* primitive reconstruction
   │
   ▼
Dense trajectory references
   │
   ▼
Multi-UGV temporal scheduler
   │
   ▼
Pure Pursuit controllers
   │
   ▼
ROS 2 / Gazebo Ackermann execution
```

---

## 1. Overview

The planning framework is built on top of **CL-CBS**:

> Wen et al., *CL-MAPF: Multi-Agent Path Finding for Car-Like Robots with Kinematic and Spatiotemporal Constraints*, Robotics and Autonomous Systems, 2022.

The conflict-resolution stage supports two interchangeable strategies:

- **`clcbs`** — CL-CBS baseline using full-horizon low-level replanning for each Body Conflict Tree branch.
- **`lcbr`** — **Local Conflict-Based Trajectory Repair**, the proposed approach, which attempts bounded local replanning around a detected conflict while retaining full-horizon replanning as fallback.

Both approaches use the same Body Conflict Tree logic and the same SHA* low-level planning machinery. The main experimental difference is therefore the replanning mechanism used after a conflict is detected.

The Gazebo component does **not** perform a second planning stage. It executes the trajectories produced by the planning framework using physical Ackermann vehicle models and closed-loop trajectory tracking.

---

## 2. Repository Structure

```text
UGV-Coordination/
│
├── README.md
├── LICENSE
├── CITATION.cff
├── .gitignore
│
├── algorithm/
│   │
│   ├── CMakeLists.txt
│   │
│   ├── include/
│   │   ├── body_conflict_tree.hpp
│   │   ├── environment.hpp
│   │   ├── hybrid_astar.hpp
│   │   ├── local_repair.hpp
│   │   ├── low_level_environment.hpp
│   │   ├── task_allocation.hpp
│   │   └── ...
│   │
│   ├── src/
│   │   └── ugv_coordination.cpp
│   │
│   ├── config/
│   │   ├── vehicle_config.yaml
│   │   └── experiments/
│   │
│   ├── experiments/
│   │   ├── main_comparison/
│   │   ├── delta_w_ablation/
│   │   ├── full_pipeline/
│   │   └── real_instances/
│   │
│   ├── analysis/
│   ├── docs/
│   ├── results/
│   ├── tests/
│   ├── tools/
│   ├── benchmark/        # local dataset, not committed
│   └── runs/             # generated solutions, not committed
│
└── simulation/
    │
    └── src/
        └── ugv_gazebo/
            │
            ├── config/
            ├── launch/
            ├── models/
            ├── resource/
            ├── scenarios/
            ├── tools/
            ├── trajectories/
            ├── ugv_gazebo/
            ├── worlds/
            ├── package.xml
            ├── setup.py
            └── setup.cfg
```

Generated ROS 2 directories

```text
simulation/build/
simulation/install/
simulation/log/
```

are intentionally excluded from version control.

The original benchmark dataset and generated experimental solutions are also kept locally and excluded from Git.

---

# Part I — Planning and Coordination

## 3. Planning Pipeline

The algorithm implements three main stages.

### Stage 1 — Nominal Trajectory Generation

For every UGV–POI pair, SHA* computes a kinematically feasible trajectory and its path length.

For \(M\) robots and \(Q\) POIs, this produces an \(M\times Q\) trajectory-cost matrix

\[
L_{ij},
\]

where \(L_{ij}\) is the SHA*-derived path cost for assigning robot \(i\) to POI \(j\).

---

### Stage 2 — Task Allocation

The assignment problem is solved using the Hungarian algorithm.

The objective is based on the actual SHA* trajectory costs rather than Euclidean distance.

The resulting robot-to-POI assignment determines the nominal trajectory set

\[
\Gamma^0.
\]

---

### Stage 3 — Multi-UGV Conflict Resolution

The nominal trajectories are checked for spatiotemporal conflicts.

Two strategies are available:

```text
--mode clcbs
```

for the CL-CBS baseline, and

```text
--mode lcbr
```

for Local Conflict-Based Trajectory Repair.

LCBR attempts to repair a conflict within a bounded local temporal window controlled by

\[
\delta_w.
\]

If the bounded repair cannot produce a valid solution, the original full-horizon query remains available as fallback.

---

## 4. Relationship to CL-CBS

| File | Status |
|---|---|
| `algorithm/include/neighbor.hpp`, `planresult.hpp`, `timer.hpp`, `hybrid_astar.hpp` | CL-CBS low-level machinery; documentation and defensive/experimental additions where noted |
| `algorithm/include/environment.hpp` | adapted for local-goal override, query instrumentation and state-validity access |
| `algorithm/include/low_level_environment.hpp` | shared low-level environment |
| `algorithm/include/task_allocation.hpp` | task-allocation and nominal-generation support |
| `algorithm/include/local_repair.hpp` | LCBR implementation |
| `algorithm/include/body_conflict_tree.hpp` | Body Conflict Tree with runtime-selectable low-level strategy |
| `algorithm/include/experiment_logger.hpp` | experiment/query logging |
| `algorithm/src/ugv_coordination.cpp` | complete Stage 1 → 2 → 3 pipeline |
| `algorithm/tools/visualize.py` | original visualization utility |
| `algorithm/tools/visualize_v2.py` | extended visualization, static plots, comparisons and GIF export |

---

## 5. Build the Algorithm

### Dependencies

On Ubuntu:

```bash
sudo apt-get install \
    g++ \
    cmake \
    libboost-program-options-dev \
    libyaml-cpp-dev \
    libompl-dev \
    libeigen3-dev
```

### Build

```bash
cd algorithm

mkdir -p build
cd build

cmake -DCMAKE_BUILD_TYPE=Release ..

make -j$(nproc)

ctest --output-on-failure
```

The main executable is:

```text
algorithm/build/ugv_coordination
```

---

## 6. Example Planner Execution

From `algorithm/`:

```bash
./build/ugv_coordination \
  --input experiments/full_pipeline/instances/paper_corridors_6ugv.yaml \
  --output runs/full_pipeline/corridors_6ugv/final_lcbr.yaml \
  --nominal-output runs/full_pipeline/corridors_6ugv/nominal.yaml \
  --mode lcbr \
  --delta_w_steps 10 \
  --timeout 600 \
  --vehicle-config config/vehicle_config.yaml
```

This produces:

```text
nominal.yaml
    └── nominal trajectories before conflict resolution

final_lcbr.yaml
    └── final coordinated trajectories after LCBR
```

---

## 7. Visualization — Before and After LCBR

### Before LCBR

```bash
python3 tools/visualize_v2.py \
  -m experiments/full_pipeline/instances/paper_corridors_6ugv.yaml \
  -s runs/full_pipeline/corridors_6ugv/nominal.yaml \
  -v runs/full_pipeline/corridors_6ugv/corridors_6ugv_BEFORE.gif \
  --speed 4
```

### After LCBR

```bash
python3 tools/visualize_v2.py \
  -m experiments/full_pipeline/instances/paper_corridors_6ugv.yaml \
  -s runs/full_pipeline/corridors_6ugv/final_lcbr.yaml \
  -v runs/full_pipeline/corridors_6ugv/corridors_6ugv_AFTER.gif \
  --speed 4
```

`--speed` changes only animation playback speed; it does not modify the planned trajectories.

---

## 8. Main Command-Line Options

| Flag | Meaning |
|---|---|
| `-i, --input` | input scenario YAML |
| `-o, --output` | final solution YAML |
| `--nominal-output` | nominal solution before conflict resolution |
| `-m, --mode` | `clcbs` or `lcbr` |
| `--delta_w_steps` | LCBR local repair margin in \(T_s\) steps |
| `--timeout` | Stage-3 wall-clock budget |
| `--max-low-level-expansions` | maximum SHA* expansions for one low-level query |
| `--cost-matrix-csv` | export the \(M\times Q\) SHA* cost matrix |
| `--assignment-csv` | export the resolved robot-to-POI assignment |
| `--instance-id` | experiment identifier |
| `--log-dir` | directory for experiment logs |
| `--vehicle-config` | vehicle/algorithm configuration |

---

# Part II — ROS 2 / Gazebo Validation

## 9. Purpose of the Simulation Layer

The simulation layer validates whether the trajectories generated by the planning framework can be executed by physical Ackermann-steered vehicle models.

The simulation does not replace SHA*, task allocation, CL-CBS or LCBR.

Instead:

```text
Planner
    decides WHERE and WHEN the UGV should move

Execution layer
    determines HOW the Ackermann vehicle follows that reference
```

The execution architecture is:

```text
final_lcbr.yaml
       │
       ▼
Exact primitive reconstruction
       │
       ▼
Dense trajectory references
       │
       ▼
Temporal multi-UGV scheduler
       │
       ▼
Pure Pursuit
       │
       ▼
(v, omega)
       │
       ▼
Gazebo Ackermann Steering
       │
       ▼
Vehicle physics
       │
       ▼
Odometry
       │
       └────────── feedback to controller
```

---

## 10. Planner-to-Simulation Interface

The main interface between the two components is:

```text
final_lcbr.yaml
```

For every agent, the trajectory contains states of the form:

```yaml
x: ...
y: ...
yaw: ...
t: ...
action: ...
```

The `action` field identifies the SHA* motion primitive.

The execution layer interprets the actions as:

| Action | Motion |
|---:|---|
| 0 | forward straight |
| 1 | forward turning primitive |
| 2 | forward turning primitive |
| 3 | reverse straight |
| 4 | reverse turning primitive |
| 5 | reverse turning primitive |
| 6 | WAIT |

---

## 11. Dense SHA* Primitive Reconstruction

The discrete planner states are not simply connected using arbitrary straight-line interpolation.

`dense_reference.py` reconstructs the geometry of the original SHA* primitives.

Run:

```bash
cd simulation/src/ugv_gazebo

python3 ugv_gazebo/dense_reference.py
```

For a turning primitive, the reconstruction uses the planner turning radius

\[
R = 3\,\text{m}.
\]

Intermediate points are sampled along the corresponding circular primitive.

For a straight primitive, intermediate points are sampled along the straight motion.

For a WAIT primitive:

\[
x_j=x_0,\qquad
y_j=y_0,\qquad
\psi_j=\psi_0,
\]

while time continues to advance.

The generated references are:

```text
trajectories/dense/
├── agent0_dense.yaml
├── agent1_dense.yaml
├── agent2_dense.yaml
├── agent3_dense.yaml
├── agent4_dense.yaml
└── agent5_dense.yaml
```

The reconstruction includes an endpoint audit to verify that each dense primitive terminates at the corresponding SHA* state.

---

## 12. Gazebo Scenario Generation

For the six-UGV corridor experiment:

```bash
cd simulation/src/ugv_gazebo

python3 tools/generate_corridors_world.py
```

This generates:

```text
worlds/paper_corridors_6ugv.sdf
```

from:

```text
scenarios/paper_corridors_6ugv.yaml
```

The current corridor scenario contains:

```text
Workspace : 55 x 55 m
Obstacles : 92
Starts    : 6
Goals     : 6
```

with the mapping:

| Planner agent | Gazebo model | Start |
|---|---|---|
| `agent0` | `ugv_1` | `(12, 4)` |
| `agent1` | `ugv_2` | `(18, 4)` |
| `agent2` | `ugv_3` | `(24, 4)` |
| `agent3` | `ugv_4` | `(30, 4)` |
| `agent4` | `ugv_5` | `(36, 4)` |
| `agent5` | `ugv_6` | `(42, 4)` |

---

## 13. Vehicle Model

Each Gazebo UGV uses an Ackermann steering model.

The principal geometric parameters are:

```text
wheel base       = 1.30 m
wheel separation = 1.16 m
wheel radius     = 0.25 m
turning radius   = 3.0 m
```

The Gazebo steering limit is chosen consistently with the planner turning radius.

Each UGV exposes independent command and odometry channels:

```text
/ugv_1/cmd_vel
/model/ugv_1/odometry

...

/ugv_6/cmd_vel
/model/ugv_6/odometry
```

---

## 14. Multi-UGV Execution

`multi_ugv_tracker.py` executes the six dense references.

The mapping is:

```text
agent0_dense.yaml -> ugv_1
agent1_dense.yaml -> ugv_2
agent2_dense.yaml -> ugv_3
agent3_dense.yaml -> ugv_4
agent4_dense.yaml -> ugv_5
agent5_dense.yaml -> ugv_6
```

Each robot uses a closed-loop Pure Pursuit controller.

The controller receives:

```text
dense reference + Gazebo odometry
```

and generates:

\[
(v,\omega).
\]

These commands are sent to the corresponding Gazebo Ackermann steering system.

---

## 15. Temporal Coordination

Geometric tracking alone is insufficient for executing a multi-robot LCBR solution because LCBR also contains temporal coordination decisions.

The execution layer therefore preserves:

- forward motion,
- reverse motion,
- WAIT actions,
- WAIT release,
- direction changes,
- temporal-frontier holds.

Gazebo odometry timestamps provide the common simulation-time basis for the six robots.

A robot that reaches a future portion of its reference too early is stopped by the temporal frontier until that trajectory segment becomes temporally admissible.

Typical execution messages include:

```text
DIRECTION SWITCH
LCBR WAIT
WAIT RELEASE
TIME FRONTIER HOLD
COMPLETE
```

---

## 16. Build the ROS 2 / Gazebo Simulation

Requirements include:

- Ubuntu
- ROS 2 Jazzy
- Gazebo / Gazebo Sim
- `ros_gz_bridge`

From the repository root:

```bash
cd simulation

source /opt/ros/jazzy/setup.bash

colcon build \
  --packages-select ugv_gazebo \
  --symlink-install

source install/setup.bash
```

Generated directories:

```text
simulation/build/
simulation/install/
simulation/log/
```

are ignored by Git.

---

## 17. Launch the Simulation

### Terminal 1 — Gazebo + ROS/Gazebo bridges

```bash
cd simulation

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ugv_gazebo simulation.launch.py
```

Wait until all six UGVs are visible.

### Terminal 2 — Multi-UGV trajectory execution

```bash
cd simulation

source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run ugv_gazebo multi_ugv_tracker
```

A successful execution terminates with:

```text
MULTI-UGV MISSION END | complete=6/6 | failed=0/6
```

---

## 18. Gazebo Simulation Speed

The world SDF contains:

```xml
<max_step_size>...</max_step_size>
<real_time_factor>...</real_time_factor>
```

`real_time_factor` specifies a target simulation acceleration.

The actual achievable real-time factor depends on the available CPU/GPU resources and rendering cost.

For performance measurements, Gazebo statistics can be inspected using:

```bash
gz topic -l | grep stats
```

and then:

```bash
gz topic -e -t /world/paper_corridors_6ugv/stats
```

For high-throughput experiments, Gazebo may also be executed in server-only/headless mode to remove GUI rendering overhead.

---

# Part III — Experiments and Reproducibility

## 19. Running the Main Experiments

From `algorithm/`:

```bash
cd experiments/main_comparison

python3 select_instances.py \
  --config ../../config/experiments/main_comparison_50x50.yaml \
  --tag 50

python3 run_comparison.py \
  --config ../../config/experiments/main_comparison_50x50.yaml \
  --tag 50
```

Repeat with the corresponding 100×100 and 300×300 configurations.

---

## 20. Repair-Window Ablation

```bash
cd algorithm/experiments/delta_w_ablation

python3 select_conflict_instances.py \
  --map-size 100by100 \
  --scenario obstacle \
  --top-n 15 \
  --tag 100conf

python3 run_ablation.py \
  --config ../../config/experiments/delta_w_ablation_100x100.yaml \
  --tag 100conf \
  --parallel 3
```

---

## 21. Solution Validation

```bash
cd algorithm/analysis

python3 validate_all_runs.py \
  --vehicle-config ../config/vehicle_config.yaml
```

The validation framework checks the generated solutions independently for the relevant kinematic, obstacle/boundary and inter-robot collision conditions.

---

## 22. Benchmark Dataset

The original Wen et al. benchmark is **not stored in this repository**.

Place it locally under:

```text
algorithm/benchmark/
```

The directory is excluded through `.gitignore`.

This avoids committing thousands of benchmark files that are publicly available upstream.

---

## 23. Generated Results

Generated experiment outputs are stored locally under:

```text
algorithm/runs/
```

and are not committed.

The repository keeps only:

```text
algorithm/runs/.gitkeep
```

Generated ROS 2 artifacts are similarly excluded:

```text
simulation/build/
simulation/install/
simulation/log/
```

The simulation package itself, reference scenario and reference trajectories remain versioned to support reproducibility.

---

## 24. Documentation

Algorithm-specific documentation is available under:

```text
algorithm/docs/
```

including:

- `LCBR.md` — algorithm-to-code mapping and LCBR details,
- `EXPERIMENTS.md` — experiment execution,
- `REPRODUCIBILITY.md` — environment, known limitations and reproducibility notes.

---

## 25. Citation

See:

```text
CITATION.cff
```

If using the underlying CL-CBS / CL-MAPF machinery, please also cite the original work by Wen et al. (2022).

Original CL-CBS repository:

https://github.com/APRIL-ZJU/CL-CBS

---

## 26. License

See:

```text
LICENSE
```

for licensing information.

---

## Project Status

Current development includes:

- SHA* nominal trajectory generation,
- trajectory-cost-based task allocation,
- CL-CBS baseline,
- Local Conflict-Based Trajectory Repair,
- experiment logging and validation,
- 50×50 / 100×100 / 300×300 experiment infrastructure,
- six-UGV ROS 2 / Gazebo execution,
- forward/reverse motion execution,
- explicit WAIT handling,
- synchronized temporal-frontier execution,
- closed-loop Pure Pursuit tracking,
- Ackermann vehicle models.

The repository is currently maintained as a research codebase associated with ongoing work.