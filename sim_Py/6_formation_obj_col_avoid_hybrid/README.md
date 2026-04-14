# Python ↔ MATLAB/Simulink Co-Simulation Platform
## Distributed Hybrid Formation Control with Unicycle Lower-Level Dynamics

---

## Overview

This module adds a **lower-level motion-control layer** implemented in
MATLAB/Simulink to the existing Python-based distributed MPC planning layer.

| Layer | Role | Key file |
|-------|------|----------|
| Planning | Distributed MPC + hybrid safety (existing) | `coordinator_node.py`, `controller_node.py` |
| Co-sim bridge | Python ↔ MATLAB interface | `cosim_bridge.py` |
| Manager | Replaces `plant_node.py`, closes the loop | `cosim_manager.py` |
| Logger | CSV + diagnostic logging | `cosim_logger.py` |
| Simulink | Unicycle model + heading controller | `matlab/unicycle_cosim.slx` |

---

## Unicycle Model

Each robot is modelled as a **differential-drive mobile robot** with the
standard unicycle kinematics:

```
ẋ     = v · cos θ
ẏ     = v · sin θ
θ̇     = ω
```

### Motion Controller

Given the desired Cartesian velocity `(u_x, u_y)` from the planning layer:

```
v_ref   = ‖[u_x, u_y]‖                   # desired linear speed
θ_ref   = atan2(u_y, u_x)               # desired heading
e_θ     = wrap(θ_ref − θ)               # heading error ∈ (−π, π]
ω       = k_heading · e_θ               # proportional heading control
v       = k_speed · v_ref · max(cos e_θ, 0)   # speed shaping
```

The `max(cos e_θ, 0)` term reduces speed when the robot is misaligned, which
prevents the robot from driving backward and improves tracking stability.

---

## Interface: Python ↔ MATLAB

Communication uses the **MATLAB Engine API for Python** (`matlab.engine`).

```
Python                        MATLAB workspace
------                        ----------------
u_ref_1 = [ux, uy]  ──→  u_ref_1   (From Workspace in Simulink)
u_ref_2 = [ux, uy]  ──→  u_ref_2
   ...                      ...
                  sim() advances by simulink_dt seconds
                  
odo_1  ←──  odo_1   (To Workspace in Simulink)  [x,y,θ,v,ω]
odo_2  ←──  odo_2
   ...
```

No additional MATLAB toolboxes are required (no Instrument Control Toolbox,
no UDP blocks).

---

## Installation

### 1. MATLAB Engine API for Python

```bash
# From your MATLAB installation root:
cd "$(matlab -batch "disp(matlabroot)")/extern/engines/python"
pip install .
```

Requires MATLAB R2021a or later.

### 2. Python dependencies

```bash
pip install pyzmq numpy
```

### 3. Directory structure

```
project/
├── consensus_config.py
├── consensus_controller.py
├── consensus_comm.py
├── explicit_hybrid_controller.py
├── coordinator_node.py
├── controller_node.py
├── cosim_config.py           ← NEW
├── cosim_bridge.py           ← NEW
├── cosim_logger.py           ← NEW
├── cosim_manager.py          ← NEW  (replaces plant_node.py)
├── cosim_experiment_runner.py← NEW
└── matlab/
    ├── build_unicycle_model.m ← NEW
    ├── cosim_step.m           ← NEW
    └── unicycle_cosim.slx    (auto-generated on first run)
```

---

## Running

### Full co-simulation (MATLAB + Simulink)

```bash
python cosim_experiment_runner.py \
    --n-agents 4 \
    --model single_integrator \
    --outer-steps 18 \
    --safety-enabled true \
    --obstacles-enabled true \
    --cosim-mode matlab \
    --simulink-dt 0.01 \
    --k-heading 4.0
```

### Dry-run (planning layer only, no MATLAB)

```bash
python cosim_experiment_runner.py \
    --n-agents 4 --model single_integrator --outer-steps 18 \
    --cosim-mode dry-run
```

---

## Output files (in `cosim_logs/`)

| File | Contents |
|------|----------|
| `planning.csv` | Per-agent planning state: r, v, u_nom, u_safe, objectives |
| `odometry.csv` | Simulink odometry: x, y, θ, v, ω + position error vs planning |
| `hybrid_modes.csv` | Hybrid mode per agent: F/C/O/CO/T, active pairs, obstacles |
| `health.csv` | Per outer-step: timing, convergence metrics, MATLAB step latency |

---

## Key tuning parameters (`cosim_config.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `simulink_dt` | `0.01` s | Simulink fixed step (must divide `planning_dt`) |
| `k_heading` | `4.0` | Proportional heading gain |
| `k_speed` | `1.0` | Speed feedforward scale |
| `max_linear_speed` | `2.0` | v saturation |
| `max_angular_speed` | `6.0` | ω saturation |

---

## Design decisions

1. **matlab.engine over UDP blocks** — avoids dependency on Instrument Control
   Toolbox; workspace variable exchange is sufficient for the simulation rate.
2. **Modular bridge** — `cosim_bridge.py` exposes only `step()` and
   `step_outer()`; swapping to a UDP implementation requires only replacing
   this file.
3. **Closed-loop feedback** — Simulink positions replace the single-integrator
   update inside the planning layer, so the planner sees the actual robot state
   rather than the ideal model.
4. **Graceful dry-run** — `--cosim-mode dry-run` lets the planning layer be
   validated without MATLAB installed.
