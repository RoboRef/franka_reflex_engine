# franka_reflex_engine

ROS 2 packages that add **safety-aware reflex behaviours** to Franka Panda manipulators,
built on top of the multimodal controller framework from
[multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2).

The core idea is a *reflex engine* that monitors the robot in real-time and transparently
switches the active controller when a hazard (collision, joint-limit, velocity-limit) is
detected — without restarting or reconfiguring `ros2_control`. 

To use the reflex engine, you just need to implement the desired nominal control behaviour (e.g. joint impedance controller) within the reflex_controllers package and desired behaviour of the reflex engine (collision/position/velocity thresholds) via .yaml file. Everything else is already taken care of by the package.

An example is provided to get an idea how everything works together under reflex_bringup package. At the moment is only available for the real hardware (Franka Panda).

---

## Package Structure

```
franka_reflex_engine/
├── reflex_controllers/      # Low-level controller library
├── reflex_engine/           # Safety state-machine library
├── reflex_mc_impl/          # ros2_control plugin 
├── reflex_bringup/          # Launch files and YAML configs
```

### `reflex_controllers`
Pure C++ library (`libreflex_controllers.so`). Implements all individual motion controllers
as subclasses of `PandaControllerInterface`:

| Controller | Behaviour |
|---|---|
| `panda_joint_impedance_controller` | Joint-space impedance (low gains → gravity-comp feel) |
| `panda_cartesian_impedance_controller` | Cartesian impedance |
| `panda_zero_g_controller` | Zero-gravity / fully backdrivable |
| `panda_super_zero_g_controller` | Reduced-gravity with extra damping |
| `panda_stop_controller` | Velocity-damping brake to hold current pose |
| `panda_move_to_start_controller` | Smooth trajectory back to a stored joint configuration |
| `panda_contact_evasion_controller` | Active contact-evasion reaction |

Also provides utilities: `RobotData` (hardware interface handles), `MotionGenerator`
(trapezoidal velocity profiles), `ControllerFactory` (string → instance lookup).

**Key dependencies:** `franka_hardware`, `franka_semantic_components`, `multi_mode_control_msgs`,
Eigen3, libfranka.

---

### `reflex_engine`
Three compiled targets: `contact_detection`, `reflex_engine`.

**`PandaContactChecker`** — reads `franka_state` at every 1 kHz cycle and computes
a 6-DOF external wrench, comparing it against configurable per-joint torque thresholds
and an EE force/torque threshold.

**`SimpleReflexEngine`** — the 4-state safety machine used by default:

```
NORMAL ──(hazard detected)─────────────────────────► REFLEX
REFLEX ──(hazard gone + cooldown elapsed)──────────► WAIT_FOR_RECOVERY
WAIT_FOR_RECOVERY ──(gesture or /recover service)──► RECOVERY
RECOVERY ──(at goal position)──────────────────────► NORMAL
```

Hazard priority (checked every cycle in `NORMAL`):
1. Joint position limit approaching → `panda_stop_controller`
2. Joint velocity hard limit exceeded → `panda_stop_controller`
3. External contact detected → `panda_zero_g_controller` (by default)

**`ReflexEngineInterface`** — abstract base; `NoReflexEngine` is the passthrough variant
(reflex disabled, useful for debugging).

**Key dependencies:** `reflex_controllers`, `franka_hardware`, `franka_msgs`,
`multi_mode_control_msgs`, `panda_motion_generator_msgs`, tf2, Eigen3, libfranka.

---

### `reflex_mc_impl`
The single `ros2_control` plugin loaded by `controller_manager`.

`ReflexMCController` owns:
- A pool of `PandaControllerInterface` instances (one per configured sub-controller)
- One `SimpleReflexEngine` per resource (arm)
- An `active_control_set_` (protected by `shared_mutex`) switched atomically at runtime

Exposes two services:

| Service | Description |
|---|---|
| `~/set_controllers` | Switch the active sub-controller at runtime |
| `~/get_controllers` | Query which sub-controller is currently active |
---

### `reflex_bringup`
Launch files and YAML configurations.

| File | Purpose |
|---|---|
| `launch/real/interactive_example.launch.py` | **Minimal real-arm launch** — the entry point |
| `config/real/interactive_example.yaml` | Controller config for the interactive example |
---

## Prerequisites & Setup

### 1. Install `multipanda_ros2`

Follow the setup instructions at
[github.com/tenfoldpaper/multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2).

This provides the required base packages:
- `franka_hardware`, `franka_description`, `franka_semantic_components`
- `franka_msgs`, `franka_robot_state_broadcaster`, `franka_gripper`
- `multi_mode_control_msgs`, `panda_motion_generator_msgs`
- `franka_multi_mode_controller` (the multimodal controller framework)

### 2. Clone and Build `franka_reflex_engine`

The `franka_reflex_engine` folder lives **inside** the `multipanda_ros2` workspace.
Build only the packages needed for the interactive example:

```bash
colcon build --packages-select \
  reflex_controllers \
  reflex_engine \
  reflex_mc_impl \
  reflex_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

## Running the Interactive Example
### Launch

```bash
ros2 launch reflex_bringup interactive_example.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

Optional arguments:

| Argument | Default | Description |
|---|---|---|
| `robot_ip` | `172.16.0.2` | IP or hostname of the Franka robot |
| `arm_id` | `panda` | Arm identifier (used in all topic/frame names) |
| `load_gripper` | `true` | Attach Franka Hand |
| `use_rviz` | `false` | Open RViz |
| `config_file` | *(bundled)* | Path to a custom controller YAML |

### What to expect

#### Default behaviour — gravity compensation

On startup the robot runs `panda_joint_impedance_controller`.

Confirm the engine state:

```bash
ros2 topic echo /panda/safety_engine_state
# → data: NORMAL
```

#### Triggering a reflex

Apply a moderate external force to the end-effector (or push a joint toward its limit).
The `SimpleReflexEngine` will detect the hazard and:

1. Immediately change to gravity compensation (same 1 kHz cycle — no gap) via `set_controllers` service
2. Progress: `NORMAL` → `REFLEX` → `WAIT_FOR_RECOVERY`
3. During `WAIT_FOR_RECOVERY` another reflex can be triggered (e.g. position) or use the haptic gesture to return to starting position

#### Recovery — haptic gesture

While in `WAIT_FOR_RECOVERY`, push the end-effector firmly (> 10 N) and hold briefly.
The engine detects the force as a recovery gesture and:

1. Activates `panda_move_to_start_controller` (state: `RECOVERY`)
2. Moves the arm smoothly back to `recovery_goal_position`
   (default: `[0, -π/4, 0, -3π/4, 0, π/2, π/4]` — a safe candle pose)
3. Returns to `NORMAL` and resumes gravity-compensation mode

#### Recovery — service call

Alternatively, trigger recovery programmatically:

```bash
ros2 service call /panda/recover std_srvs/srv/Empty
```

### Tuning

All thresholds and gains live in
`reflex_bringup/config/real/interactive_example.yaml` and take effect after a rebuild
(no recompile needed — only `colcon build --packages-select reflex_bringup`).

| Parameter | Default | Effect |
|---|---|---|
| `F_max_EE` | `15.0` N | Lower to trigger reflex with lighter contact |
| `tau_max` | `[12,12,9,9,6,6,4]` Nm | Per-joint torque collision threshold |
| `gesture_threshold` | `15.0` N | Force needed to confirm recovery gesture |
| `recovery_goal_position` | candle pose | 7-DOF joint pose to return to after recovery |
| `enable_gesture_recovery` | `true` | Set `false` to require a service call instead |

---

## Architecture Diagram

```
controller_manager
  └── ReflexMCController  (reflex_mc_impl)
        ├── RobotData[panda]  ← franka_hardware state/command interfaces
        ├── control_map_
        │     ├── panda_joint_impedance_controller  ─┐
        │     ├── panda_zero_g_controller             │  PandaControllerInterface
        │     ├── panda_stop_controller               │  (reflex_controllers)
        │     └── panda_move_to_start_controller    ─┘
        └── SimpleReflexEngine[panda]  (reflex_engine)
              ├── PandaContactChecker  ─── reads franka_state @ 1 kHz
              └── state machine: NORMAL → REFLEX → WAIT_FOR_RECOVERY → RECOVERY → NORMAL
                    calls set_controllers on itself to switch the active sub-controller
```
## Implementation Plan:
- Interactive example with pose publisher
- Reflex behaviour in simulation + Demo
- Documentation: How to integrate own workflows within reflex engine
- Add more reflex behaviours (task perserving)
- Extend the simple reflex engine to two levels: Task perserving and Task abandoning