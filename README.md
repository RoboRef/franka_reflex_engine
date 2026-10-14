# franka_reflex_engine

**Author:** Jon Skerlj — [jon.skerlj@gmail.com](mailto:jon.skerlj@gmail.com)

ROS 2 packages that add **safe robot reflex behaviours** to Franka Panda manipulators, built on top of the multimodal controller framework from [multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2).

The framework implements the reflex behaviours proposed in:
> **J. Vorndamme, A. Melone, R. Kirschner, L. Figueredo and S. Haddadin,**, 
> *"Safe Robot Reflexes: A Taxonomy-Based Decision and Modulation Framework,"*  IEEE Transactions on Robotics (T-RO), 2025.  
> **Paper link:** [https://ieeexplore.ieee.org/document/10804590](https://ieeexplore.ieee.org/document/10804590)

The core idea is a real-time **reflex engine** that monitors the robot at 1 kHz and transparently switches the active controller when a hazard (collision, joint position limit, joint velocity limit) is detected — without restarting or reconfiguring `ros2_control` by utilizing the fast swiching capabilities of multimodal controller as described in:
> **J. Škerlj, S. Bien, A. Naceri and S. Haddadin,**, 
> *"Bridging the Sim-to-Real Gap with multipanda_ros2: A Real-Time ROS 2 Framework for Multimanual Systems,"*  2026 IEEE International Conference on Robotics and Automation (ICRA).  
> **Paper link:** [https://ieeexplore.ieee.org/document/11696149](https://ieeexplore.ieee.org/document/11696149)
---

## Package Structure

```
franka_reflex_engine/
├── reflex_bringup/          # Launch files and YAML configs for examples
├── reflex_controllers/      # Safety reflex behaviours & framework core
├── reflex_engine/           # Safety state-machine library
└── reflex_mc_impl/          # ros2_control plugin (ReflexMultimodeController)
├── task_controllers/        # Task controllers
```

### 1. `reflex_bringup` (Examples)
Provides configurations and launch files for the available demonstrations:
- **`interactive_example`**: Gravity-compensation interaction mode with active collision and joint limit reflex monitoring.
- **`figure_eight_example`**: Autonomous trajectory tracking (lemniscate on the base XY plane) with collision/limit reflex reaction and automated recovery.

### 2. `reflex_controllers`
Implements the core `PandaControllerInterface` along with the **task-abandoning reflex behaviours**:

| Reflex | Controller | Behaviour / Control Law |
|---|---|---|
| **Stop** | `franka_stop_controller` | Freezes at current position with high stiffness and smooth velocity ramp-down: $$q_d^*(t) = q(t_{cd})$$ |
| **Zero-G** | `franka_zero_g_controller` | Weightless and backdrivable with protective soft joint-limit walls: $$\tau =\tau_g(t)$$ |
| **Super Zero-G** | `franka_super_zero_g_controller` | Active assistance to feel lighter than zero-G: $$\tau =\tau_g(t) -\beta \tau_{ext},\text{with }\beta = 0.5$$ |
| **Admittance** | `franka_admittance_controller` | Compliant yielding: $$ \dot{q}^*_d(t) = \alpha \hat{\tau}_{ext}(t) with \alpha = 0.05 $$ |
| **Joint Retraction** | `franka_joint_retract_controller` | Smooth quintic retraction away from impact torque: $$q_d^*(t) = q(t_{cd}) + \frac{\delta \tau_{ext}(t_{cd})}{\lVert J_{Cart} \tau_{ext}(t_{cd}) \rVert} \text{with }\delta = 0.05$$ |
| **Cartesian Retraction** | `franka_cartesian_retract_controller` | Smooth Cartesian retraction away from contact force: $$x_d^*(t) = x(t_{cd}) + \begin{pmatrix} \frac{\gamma f_{ext}(t_{cd})}{\lVert f_{ext}(t_{cd}) \rVert} \\ 0 \end{pmatrix} \text{with }\gamma = 0.09 $$ |

### 3. `task_controllers`
Implements nominal task behaviours:
- `franka_joint_impedance_controller`: Joint-space impedance control
- `franka_cartesian_impedance_controller`: 6-DOF Cartesian impedance control
- `franka_joint_velocity_controller`: Joint velocity interface controller
- `franka_move_to_start_controller`: Smooth quintic trajectory generator to start / home pose
- `wiping_controller` / `comless_wiping_controller`: Harmonic surface wiping motion with active Z force control

### 4. `reflex_engine`
Contains the real-time safety monitoring state machine:
- **`PandaContactChecker`**: Computes estimated contact wrenches and evaluates collision, joint limit, and haptic gesture thresholds at 1 kHz.
- **`SimpleReflexEngine`**: Manages safety lifecycle transitions:
  ```
  NORMAL ──(hazard detected)─────────────────────────► REFLEX
  REFLEX ──(safety controller active)────────────────► WAIT_FOR_RECOVERY
  WAIT_FOR_RECOVERY ──(gesture or /recover service)──► RECOVERY
  RECOVERY ──(at home position)──────────────────────► NORMAL
  ```

### 5. `reflex_mc_impl`
The `ros2_control` controller plugin (`ReflexMultimodeController`) loaded by `controller_manager`. Exposes runtime sub-controller switching services:
- `~/set_controllers`: Switch active sub-controllers
- `~/get_controllers`: Query active sub-controllers

---

## Prerequisites & Build

### 1. Install `multipanda_ros2`
Follow the setup instructions at [github.com/tenfoldpaper/multipanda_ros2](https://github.com/tenfoldpaper/multipanda_ros2) via the one-click installer utilizing docker.

### 2. Build with `--symlink-install`
Using `--symlink-install` links YAML configuration files, launch scripts, and Python nodes directly to the install directory. This allows editing parameters, tuning thresholds, or swapping reflex behaviours without having to recompile each time:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Examples

### 1. Figure 8 Example
The robot first moves to its start configuration (`franka_move_to_start_controller`), captures the current end-effector pose as the center, and traces a continuous figure-8 trajectory (lemniscate of Gerono) in the base XY plane using `franka_cartesian_impedance_controller`.

Works for both real hardware and MuJoCo simulation.

#### Launch on Real Robot:
```bash
ros2 launch reflex_bringup figure_eight_example.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

#### Launch in MuJoCo Simulation:
```bash
ros2 launch reflex_bringup figure_eight_sim.launch.py
```

- **Default Reflex:** `zero_g` (`franka_zero_g_controller`). Can be changed in [`figure_eight_example.yaml`](reflex_bringup/config/real/figure_eight_example.yaml) or [`figure_eight_sim_controllers.yaml`](reflex_bringup/config/sim/figure_eight_sim_controllers.yaml) via `reflexes.reflex_for_collision`.
- **Triggering the Reflex:**
  - **In Simulation:** In the MuJoCo viewer window, double-click a robot body/link and hold `Ctrl` + drag with the mouse to apply external disturbance forces to the arm.
  - **On Real Hardware:** Gently push or bump into any part of the manipulator arm.
- **Recovery:**
  - **Real Robot:** A **haptic tap** (> 15 N on the end-effector) works best to automatically trigger recovery and resume the figure-8 task.
  - **Simulation:** Call the recovery service:
    ```bash
    ros2 service call /panda/recover std_srvs/srv/Empty {}
    ```

---

### 2. Interactive Example
Puts the robot into compliant gravity-compensation mode (`franka_joint_impedance_controller` with low stiffness), allowing physical interaction, guiding, and manual trigger testing of safety reflexes upon collision or limit violation. This example is easiest to explore with the real robot.

#### Launch on Real Robot:
```bash
ros2 launch reflex_bringup interactive_example.launch.py robot_ip:=<YOUR_ROBOT_IP>
```

#### Launch in MuJoCo Simulation:
```bash
ros2 launch reflex_bringup franka_sim.launch.py
```

- **Default Reflex:** `zero_g` (`franka_zero_g_controller`). Can be changed by modifying `reflexes.reflex_for_collision` in [`interactive_example.yaml`](reflex_bringup/config/real/interactive_example.yaml) or [`single_sim_controllers.yaml`](reflex_bringup/config/sim/single_sim_controllers.yaml).
- **Available Reflex Choices:**
  - `franka_zero_g_controller` (Zero-G compliant float)
  - `franka_super_zero_g_controller` (Super Zero-G active assistance)
  - `franka_stop_controller` (Instant high-stiffness freeze)
  - `franka_admittance_controller` (Compliant yielding away from force)
  - `franka_joint_retract_controller` (Joint-space retraction away from impact)
  - `franka_cartesian_retract_controller` (Cartesian retraction away from impact)
- **Recovery:** Recover via haptic tap (> 15 N on EE) or service call:
  ```bash
  ros2 service call /panda/recover std_srvs/srv/Empty {}
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

---

## Development Plan

- [x] ~~**Integration of task abandoning reflexes:**~~
  - [x] ~~`zero_g`, `super_zero_g`, `stop`, `admittance`, `joint_retract`, `cartesian_retract`~~
- [x] ~~**Demo examples:**~~
  - [x] ~~Interactive and Figure 8 motion~~
- [ ] **Documentation - How to add reflexes to your own controllers**
- [ ] **Integration of task relaxing and task preserving reflexes:**
  - Motion / Force Adaptation
  - Impedance Reaction
  - Phase Scaling
  - Nullspace Motion
  - Preserving End-Effector Orientation
- [ ] **Contact detection, isolation, and identification:**
  - Extension beyond simple threshold values (momentum observers, model-based isolation)
- [ ] **Extension to dual/multi-arm setups**