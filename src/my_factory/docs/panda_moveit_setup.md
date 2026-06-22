# Panda Manipulator + MoveIt in Ignition Gazebo

This document describes how the Franka Emika **Panda** arm was added to the
`my_factory` package so it can be simulated under **real Ignition (Fortress)
physics** and controlled through **MoveIt 2**, reusing the same warehouse world
(`fac.world`) that the mobile robot used.

---

## 1. Overview / How it functions

The setup chains four pieces together:

```
                +-------------------------------------------------------------+
                |                  Ignition Gazebo (fac.world)               |
                |                                                             |
                |   Panda model  ── ign_ros2_control-system plugin           |
                |                       │  (runs a controller_manager         |
                |                       │   INSIDE the simulator)             |
                +-----------------------│-------------------------------------+
                                        │ loads panda_controllers.yaml
                 ros2_control hardware  │ = IgnitionSystem
                                        ▼
        joint_state_broadcaster   panda_arm_controller    panda_hand_controller
                 (/joint_states)   (FollowJointTrajectory)   (GripperCommand)
                        │                    ▲                      ▲
                        ▼                    │ follow_joint_trajectory / gripper_cmd
            robot_state_publisher            │
               (TF tree)                     │
                        │                    │
                        ▼                    │
                    MoveIt move_group  ──────┘   (plans, then executes via the
                        │                          moveit_simple_controller_manager)
                        ▼
                      RViz  (MotionPlanning panel)
```

Key idea: unlike the typical MoveIt demo that uses `mock_components` (a fake
hardware that just echoes commands), here the hardware interface is
**`ign_ros2_control/IgnitionSystem`**. The `controller_manager` is launched
*inside* Gazebo by the `ign_ros2_control-system` plugin, so the joint commands
actually drive the simulated physics, and the joint states come back from the
physics engine.

The flow for a motion:

1. You set a goal in the RViz **MotionPlanning** panel (or send a goal to
   `move_group`).
2. `move_group` plans with OMPL and validates against the SRDF/collision model.
3. Execution is handed to `moveit_simple_controller_manager`, which forwards the
   trajectory to the matching `ros2_control` controller:
   - arm  → `panda_arm_controller` via a `FollowJointTrajectory` action.
   - hand → `panda_hand_controller` via a `GripperCommand` action.
4. Those controllers write position commands to the `IgnitionSystem` hardware,
   which moves the joints in the simulator.
5. `joint_state_broadcaster` publishes `/joint_states`; `robot_state_publisher`
   turns that into the TF tree; RViz and MoveIt see the robot move.

---

## 2. Why the stock Panda description wasn't enough

The MoveIt resource package `moveit_resources_panda_description` is meant for
**visualization / kinematics only**:

- Its links have **no `<inertial>` blocks** → a physics engine can't simulate
  them (zero mass/inertia is invalid).
- It expects a **floating** virtual base (it is normally pinned by MoveIt's
  planning scene, not by physics).
- Its `ros2_control` macros only ship a `mock_components` / `isaac` hardware,
  not the Ignition one.

So a dedicated, physics-ready description was authored: `panda_ign.urdf.xacro`.

---

## 3. Files and their roles

All paths are relative to `src/my_factory/`.

### `urdf/panda_ign.urdf.xacro`  — the robot description

Self-contained Panda description used by **both** Gazebo and MoveIt. It:

- Re-declares the full kinematic chain (`panda_link0` … `panda_link8`,
  `panda_hand`, `panda_leftfinger`, `panda_rightfinger`) reusing the stock
  visual `.dae` and collision `.stl` meshes from
  `moveit_resources_panda_description`, with the exact joint origins/limits.
- Adds approximate Franka **`<inertial>`** values to every link plus joint
  `<dynamics damping=…>` so the arm is stable under physics.
- Pins the base: a `world` link and a fixed `panda_world_joint`
  (`world → panda_link0`) so the arm stays anchored to the floor at the origin
  instead of falling.
- Declares one **`ros2_control`** block of type `system` using
  `ign_ros2_control/IgnitionSystem`, listing:
  - the 7 arm joints (`position` command; `position`/`velocity`/`effort` state),
  - `panda_finger_joint1` (the actuated gripper joint),
  - `panda_finger_joint2` as a **mimic** of `panda_finger_joint1`
    (`<param name="mimic">` + `<param name="multiplier">1</param>`).
- Adds the `<gazebo>` `ign_ros2_control-system` plugin, pointing
  `<parameters>` at `config/panda_controllers.yaml` via `$(find my_factory)`.

> **Mimic note:** this build of `ign_ros2_control` builds its mimic list from the
> `<param name="mimic">` inside the `ros2_control` block — it does **not** read
> the URDF `<mimic>` tag. The mimic joint therefore *must* be declared in the
> control block, otherwise only one finger moves.

### `config/panda_controllers.yaml`  — ros2_control controllers

Loaded by the `ign_ros2_control-system` plugin. Defines the `controller_manager`
(100 Hz, `use_sim_time: true`) and three controllers:

| Controller | Type | Joints |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | publishes only the real joints (7 arm + `panda_finger_joint1`) |
| `panda_arm_controller` | `joint_trajectory_controller/JointTrajectoryController` | `panda_joint1..7` |
| `panda_hand_controller` | `position_controllers/GripperActionController` | `panda_finger_joint1` |

> The `joint_state_broadcaster` `joints:` list is restricted on purpose: the
> Ignition plugin exposes the mimic joint as `panda_finger_joint2_mimic`. If it
> were broadcast, MoveIt would spam
> `Joint 'panda_finger_joint2_mimic' not found in model`. We don't broadcast it;
> `robot_state_publisher`/MoveIt recompute `panda_finger_joint2` from the URDF
> mimic relationship.

### `config/moveit_controllers.yaml`  — MoveIt → ros2_control mapping

Tells MoveIt's `moveit_simple_controller_manager` how to *execute* a plan. The
stock Panda file only maps the arm, which is why the **hand group originally
failed to execute**. This local copy registers both:

- `panda_arm_controller` → `action_ns: follow_joint_trajectory`, type
  `FollowJointTrajectory`.
- `panda_hand_controller` → `action_ns: gripper_cmd`, type `GripperCommand`,
  joint `panda_finger_joint1`.

### `launch/panda_moveit.launch.py`  — the bringup

Single entry point. It:

1. Processes `panda_ign.urdf.xacro` into `robot_description`.
2. Builds the MoveIt config with `MoveItConfigsBuilder`, reusing the stock
   Panda **SRDF**, kinematics and OMPL pipeline, but overriding
   `robot_description` with our xacro and `trajectory_execution` with our
   `config/moveit_controllers.yaml`.
3. Starts Ignition with `worlds/fac.world`.
4. Starts `robot_state_publisher`, spawns the Panda (`ros_gz_sim create`),
   and bridges `/clock`.
5. After the model spawns, loads the controllers in order
   (`joint_state_broadcaster` → `panda_arm_controller` → `panda_hand_controller`)
   using `controller_manager` spawners and `OnProcessExit` event handlers.
6. Starts `move_group` and `RViz` (with the MotionPlanning plugin), all with
   `use_sim_time: true`.

### Reused, unmodified assets (from `moveit_resources_panda_moveit_config`)

- `config/panda.srdf` — planning groups (`panda_arm`, `hand`), collision rules,
  named states (`ready`, `open`, `close`).
- kinematics (KDL), joint limits, OMPL planning config, and `launch/moveit.rviz`.

---

## 4. Dependencies

Added to `package.xml`:

```
moveit_ros_move_group, moveit_configs_utils,
moveit_resources_panda_moveit_config, moveit_resources_panda_description,
ros_gz_sim, controller_manager, joint_trajectory_controller,
joint_state_broadcaster, position_controllers
```

Plus an `exec_depend` on **`ign_ros2_control`**, which lives in the workspace
pointed to by **`ABDO_EXTRA_WS`** (see `config/abdo.env.example`) and must be
sourced at runtime.

---

## 5. How to build & launch

Each developer sets workspace paths once (see repo `README.md` and
`config/abdo.env.example`):

```bash
cp config/abdo.env.example config/abdo.env   # set ABDO_WS / ABDO_EXTRA_WS
source ../../env/setup_abdo.bash

colcon build --packages-select my_factory --symlink-install
ros2 launch my_factory panda_moveit.launch.py
```

Package assets (URDF, meshes, configs) are resolved by ROS 2 packaging —
only **`ABDO_WS`** (this repo's colcon root) and **`ABDO_EXTRA_WS`**
(`ign_ros2_control` workspace) vary per machine.

In RViz, use the **MotionPlanning** panel: pick the `panda_arm` group, drag the
interactive marker (or choose a named state like `ready`), then *Plan & Execute*.
For the gripper, select the `hand` group and use the `open` / `close` states.

---

## 6. Sanity checks / troubleshooting

| Check | Command | Expected |
|---|---|---|
| Controllers active | `ros2 control list_controllers` | all three `active` |
| Gripper action up | `ros2 action list \| grep gripper_cmd` | `/panda_hand_controller/gripper_cmd` |
| Mimic registered | look in launch log | `Joint 'panda_finger_joint2' is mimicking joint 'panda_finger_joint1'` |
| Clean joint states | `ros2 topic echo /joint_states --once` | lists `panda_finger_joint1`, **no** `..._mimic` |

Common pitfalls already handled here, for reference:

- **Hand group won't execute** → MoveIt had no controller mapping for it; fixed
  by adding `panda_hand_controller` (`GripperCommand`) to
  `config/moveit_controllers.yaml`.
- **Only one finger moves** → mimic must be declared via `<param name="mimic">`
  in the `ros2_control` block (the URDF `<mimic>` tag alone is ignored by this
  plugin version).
- **`panda_finger_joint2_mimic not found in model` spam** → restrict the
  `joint_state_broadcaster` `joints:` list so the suffixed mimic interface isn't
  published.
- **Plugin/controllers don't start** → you forgot to source the `ros2_ws` that
  provides `ign_ros2_control`.
