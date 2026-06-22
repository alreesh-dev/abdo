# ABDO simulation workspace (ROS 2)

Warehouse mobile-robot simulation (Nav2 + SLAM + EKF) and Panda manipulator
simulation (MoveIt + `ign_ros2_control`) in Ignition Gazebo.

Packages:

- **`my_factory`** — worlds, robots, launch files, Nav2/Panda configs
- **`ira_laser_tools`** — laser scan merger (optional)

## First-time setup (each developer)

1. Clone this repo into your colcon workspace `src/` folder.

2. Copy the environment template and set **your** workspace paths (the only
   machine-specific settings — package/mesh paths use standard ROS 2 packaging):

   ```bash
   cp my_factory/config/abdo.env.example my_factory/config/abdo.env
   # edit ABDO_WS and ABDO_EXTRA_WS
   ```

3. Build:

   ```bash
   cd "$ABDO_WS"
   colcon build --symlink-install
   ```

4. Source:

   ```bash
   source src/env/setup_abdo.bash
   ```

## Environment variables (workspace paths only)

| Variable | Purpose | Default |
|---|---|---|
| `ABDO_WS` | Colcon workspace root (contains `install/setup.bash`) | `$HOME/Desktop/assist` |
| `ABDO_EXTRA_WS` | Extra workspace with `ign_ros2_control` (Panda sim) | `$HOME/ros2_workspaces/ros2_ws` |

Everything else (maps, URDF, configs, Panda meshes) is resolved via
`get_package_share_directory`, `package://`, and `$(find my_factory)`.

`my_factory/config/abdo.env` is git-ignored; only `abdo.env.example` is tracked.

## Launch commands

Mobile robot — SLAM / mapping:

```bash
ros2 launch my_factory main_launch.py
```

Mobile robot — Nav2 in warehouse map:

```bash
ros2 launch my_factory main_warehouse_nav.launch.py
```

Panda arm — MoveIt in same warehouse world:

```bash
ros2 launch my_factory panda_moveit.launch.py
```

## Docs

- `my_factory/docs/diff_drive_fix.md` — mobile robot tuning & localization fixes
- `my_factory/docs/panda_moveit_setup.md` — Panda + MoveIt + ign_ros2_control setup

## External dependencies

- ROS 2 Humble
- Ignition Gazebo (Fortress) via `ros_gz_sim`
- Nav2, SLAM Toolbox, `robot_localization` (mobile stack)
- MoveIt 2 + `moveit_resources_panda_*` (Panda stack)
- **`ign_ros2_control`** in a separate colcon workspace (`ABDO_EXTRA_WS`)
