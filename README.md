# franka_reflex_engine

**Author:** Jon Skerlj — [jon.skerlj@gmail.com](mailto:jon.skerlj@gmail.com)

ROS 2 packages that add **safety-aware reflex behaviours** to Franka Panda manipulators,
built on top of the multimodal controller framework from
[multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2).

The core idea is a *reflex engine* that monitors the robot in real-time and transparently
switches the active controller when a hazard (collision, joint-limit, velocity-limit) is
detected — without restarting or reconfiguring `ros2_control`. 

---

## Package Structure

```
franka_reflex_engine/
├── reflex_controllers/      # Safety reflex behaviours & framework core
├── task_controllers/        # Nominal / task motion controllers
├── reflex_engine/           # Safety state-machine library (SimpleReflexEngine)
├── reflex_mc_impl/          # ros2_control plugin (ReflexMultimodeController)
└── reflex_bringup/          # Launch files and YAML configs
```

### 1. `reflex_controllers`
Implements the core `PandaControllerInterface` framework along with the **5 task-abandoning reflex behaviours** (based on Vorndamme et al., IEEE T-RO 2025):

| Reflex | Controller | Behaviour / Control Law |
|---|---|---|
| **Stop** | `franka_stop_controller` | Freezes at current position with high stiffness and smooth velocity ramp-down |
| **Zero-G** | `franka_zero_g_controller` | Weightless and backdrivable with protective soft joint-limit walls |
| **Super Zero-G** | `franka_super_zero_g_controller` | Active assistance ($\tau = -K_r \tau_{ext}$) to feel lighter than zero-G |
| **Admittance** | `franka_admittance_controller` | Compliant yielding: $\dot{q}^*_d(t) = \alpha \hat{\tau}_{ext}(t)$ with $\alpha = 0.05$ |
| **Joint Retraction** | `franka_joint_retract_controller` | Smooth quintic retraction away from impact torque: $q_{goal} = q_0 - \Delta \frac{\hat{\tau}_c}{\|\hat{\tau}_c\|}$ |
| **Cartesian Retraction** | `franka_cartesian_retract_controller` | Smooth Cartesian retraction away from contact force: $p_{goal} = p_0 - d \frac{f_c}{\|f_c\|}$ |

### 2. `task_controllers`
Implements nominal task behaviours:
- `franka_joint_impedance_controller`: Joint-space impedance (low stiffness for gravity-comp feel)
- `franka_cartesian_impedance_controller`: 6-DOF Cartesian impedance control
- `franka_joint_velocity_controller`: Joint velocity interface controller
- `franka_move_to_start_controller`: Smooth trajectory generator returning to start / home pose
- `wiping_controller` / `comless_wiping_controller`: Harmonic surface wiping motion with active Z force control

### 3. `reflex_engine`
Contains the real-time safety monitoring state machine:
- **`PandaContactChecker`**: Computes estimated contact wrenches and detects collision, joint limit, and haptic gesture events at 1 kHz.
- **`SimpleReflexEngine`**: Manages state transitions:
  ```
  NORMAL ──(hazard detected)─────────────────────────► REFLEX
  REFLEX ──(safety controller active)────────────────► WAIT_FOR_RECOVERY
  WAIT_FOR_RECOVERY ──(gesture or /recover service)──► RECOVERY
  RECOVERY ──(at home position)──────────────────────► NORMAL
  ```

### 4. `reflex_mc_impl`
The `ros2_control` controller plugin (`ReflexMultimodeController` / `ReflexMCController`) loaded by `controller_manager`. Exposes runtime sub-controller switching services:
- `~/set_controllers`: Switch active sub-controllers
- `~/get_controllers`: Query active sub-controllers

### 5. `reflex_bringup`
Launch files and YAML configurations:
- `launch/real/interactive_example.launch.py`: Launch for real Franka Panda hardware
- `config/real/interactive_example.yaml`: Parameter configuration (nominal + reflex selection)
- `launch/sim/franka_sim.launch.py`: MuJoCo simulation launch
- `config/sim/single_sim_controllers.yaml`: Simulation controller parameters

---

## Prerequisites & Setup

### 1. Install `multipanda_ros2`
Follow setup instructions at [github.com/tenfoldpaper/multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2).

### 2. Build `franka_reflex_engine`

```bash
colcon build --packages-select \
  reflex_controllers \
  task_controllers \
  reflex_engine \
  reflex_mc_impl \
  reflex_bringup \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

---

## Running the Interactive Example

### Launch on Hardware

```bash
ros2 launch reflex_bringup interactive_example.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

### Launch in MuJoCo Simulation

```bash
ros2 launch reflex_bringup franka_sim.launch.py
```

### Switching Reflex Behaviours
To select any of the 5 task-abandoning reflexes, set `reflexes.reflex_for_collision` in `interactive_example.yaml` (or `single_sim_controllers.yaml`):
- `franka_zero_g_controller` (Zero-G)
- `franka_admittance_controller` (Admittance)
- `franka_joint_retract_controller` (Joint Retraction)
- `franka_cartesian_retract_controller` (Cartesian Retraction)
- `franka_stop_controller` (Stop)

### Recovery
In `WAIT_FOR_RECOVERY`, recover via haptic tap (> 15 N on EE) or service call:
```bash
ros2 service call /panda/recover std_srvs/srv/Empty
```

---

## Architecture Diagram

```
controller_manager
  └── ReflexMultimodeController  (reflex_mc_impl)
        ├── RobotData[panda]  ← franka_hardware handles
        ├── control_map_
        │     ├── task_controllers:   [joint_impedance, cartesian_impedance, move_to_start, wiping]
        │     └── reflex_controllers: [stop, zero_g, super_zero_g, admittance, joint_retract, cartesian_retract]
        └── SimpleReflexEngine[panda]  (reflex_engine)
              ├── PandaContactChecker  ─── reads franka_state @ 1 kHz
              └── state machine: NORMAL → REFLEX → WAIT_FOR_RECOVERY → RECOVERY → NORMAL
```

## To Do:
- Integration of task preserving reflexes
- Example task in simulation