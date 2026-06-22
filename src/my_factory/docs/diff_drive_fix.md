# DiffDrive Controller Fix — Slow Forward Movement

## Problem

In Gazebo, the robot (`my_robot`) moved **forward very slowly** even though Nav2 was
commanding a normal speed (`max_vel_x: 0.5 m/s` in `config/nav2_params.yaml`).

The robot behaved as if it was crawling, never reaching its commanded velocity.

## Root Cause

The issue was **not** in the robot structure (`mobile_base.xacro`) or in Nav2 — it was
in the DiffDrive plugin parameters inside `urdf/gazebo_control.xacro`.

### 1. Crippling acceleration limits (main cause)

```xml
<max_linear_acceleration>0.1</max_linear_acceleration>
<max_wheel_acceleration>0.1</max_wheel_acceleration>
```

- `max_linear_acceleration = 0.1 m/s²` meant it took **5 full seconds** of continuous
  forward command just to reach 0.5 m/s. Since Nav2 constantly re-issues and adjusts
  velocity commands, the robot never sustained a single command long enough to ramp up
  — so it just crawled.
- `max_wheel_acceleration = 0.1 rad/s²` throttled the wheels just as hard. At a wheel
  radius of `0.15 m`, that is only `0.015 m/s²` of ground acceleration per wheel.

### 2. Wrong wheel separation

```xml
<wheel_separation>3.8</wheel_separation>
```

The wheels are mounted at `wheel_offset_y = 1.05` (in `mobile_base.xacro`), one on each
side, so the **actual track width is `2 × 1.05 = 2.1 m`**, not `3.8`. A wrong value does
not affect straight-line speed, but it corrupts turning behavior and odometry, which can
confuse Nav2.

## Changes Made

All changes were applied to `urdf/gazebo_control.xacro`:

| Parameter | Before | After | Reason |
|---|---|---|---|
| `max_linear_acceleration` | `0.1` | `1.0` | Reach 0.5 m/s in ~0.5 s instead of 5 s — fixes the slow crawl |
| `max_wheel_acceleration` | `0.1` | `2.0` | Allow the wheels to spin up at a realistic rate |
| `wheel_separation` | `3.8` | `2.1` | Match actual track width (2 × `wheel_offset_y`) → correct turning & odometry |
| `max_wheel_torque` | `30` | `60` | Extra headroom for the ~63 kg robot to start and climb cleanly |
| `max_angular_acceleration` | `0.5` | `1.5` | Snappier, consistent rotation |

`wheel_diameter = 0.3` was left unchanged because it was already correct
(radius `0.15 m` → diameter `0.3 m`).

## How to Apply

Rebuild and re-source so the updated xacro is picked up, then relaunch:

```bash
colcon build --packages-select my_factory && source install/setup.bash
```

## Result

The robot now accelerates forward normally and reaches its commanded velocity, with
correct turning and odometry. ✅

## Notes / Future Tuning

- If the robot ever struggles to start moving or to climb inclines, increase
  `max_wheel_torque` toward `100`.
- Wheel friction (`mu1`/`mu2 = 100` in `mobile_base.xacro`) adds scrubbing resistance
  during turns due to the 4-wheel rigid layout, but it does not block straight-line
  motion.

---

# Navigation & Localization Fixes

Follow-up issues found while bringing up Nav2 and the laser/localization stack.

## 1. Topic name mismatch (odom / TF)

The `ros_gz` bridge expected `/model/warehouse_robot/{odometry,tf}`, but the DiffDrive
plugin overrode them. Fixed in `gazebo_control.xacro`: removed the `odom_topic` override
so it uses the default model-namespaced topic the bridge listens to. (`cmd_vel` already
matched.)

## 2. Dead `~/abdo` paths

Several files hard-coded a non-existent `~/abdo/...` / `/home/abdulrahman/abdo/...` path:

- `main_warehouse_nav.launch.py` — map + behavior-tree paths → now use `pkg_path`.
- `maps/warehouse_to_edit.yaml` — `image:` → changed to the **relative** `warehouse_to_edit.pgm`
  (this was the blocker that left `map_server`/`amcl` stuck `unconfigured`).

## 3. `my_task.py` not runnable / Nav2 not active

- Made `scripts/my_task.py` executable (`chmod +x`) so `ros2 run` lists it.
- Gave `BasicNavigator` its own node name (removed duplicate-name warning).
- The mission requires `main_warehouse_nav.launch.py` (has AMCL); `main_launch.py` is SLAM
  only (no AMCL → `amcl/get_state` hang).

## 4. Laser scan not matching map

- **`transform_tolerance: 5.0 → 0.3`** in `amcl` + both costmaps — 5 s let Nav2 reuse a
  stale pose, so the scan swung off the walls while turning.
- Navigation now uses the **single front-right lidar** (`/front_right_scan → /scan`) to match
  how the map was built; removed the `laserscan_multi_merger`.

## 5. Skid-steer drift → IMU + EKF fusion

Root cause: 4-wheel skid-steer + high lateral friction makes wheel-odometry rotation
unreliable, so heading drifted during turns. Fix = fuse wheel odom + IMU via
`robot_localization`:

- `gazebo_control.xacro`: `publish_odom_tf: true → false` (EKF now owns `odom → base_footprint`).
- `mobile_base.xacro`: IMU on `/imu` with `gz_frame_id: imu_link`.
- `config/ekf.yaml` (new): wheel `vx` only + IMU yaw/yaw-rate, `two_d_mode`.
- `main_warehouse_nav.launch.py` + `main_launch.py`: added `/imu` bridge and `ekf_node`,
  removed the Gazebo `odom→base` TF bridge.
- AMCL motion noise raised (`alpha1: 0.8`, `alpha2`/`alpha4: 0.4`, `update_min_a: 0.1`).
- Added `robot_localization` to `package.xml`.

TF ownership: `map→odom` = AMCL, `odom→base_footprint` = EKF, static links = robot_state_publisher.

## Rebuild gotcha

Adding the **new** `config/ekf.yaml` required a `colcon build` — `--symlink-install` only
links files that existed at build time. Skipping it left the EKF with no config (no
subscriptions, no output), which cascaded into `"map" frame does not exist`. Always
rebuild once after adding a new file:

```bash
colcon build --packages-select my_factory --symlink-install && source install/setup.bash
```
