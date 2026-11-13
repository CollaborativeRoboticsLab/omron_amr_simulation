# Omron AMR Gazebo simulation

ROS 2 Jazzy / Gazebo Harmonic starting point for the LD90 and LD250. The current
launch is a **static geometry preview**, not a driveable or sensor-equipped robot.
It does not start the real AMR driver or connect to a robot.

## Build and run

Install the ROS dependencies with rosdep, then build this package and the existing
`amr_description` package in the same workspace:

```bash
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to omron_amr_simulation
source install/setup.bash
ros2 launch omron_amr_simulation preview.launch.py model:=ld250
```

Use `model:=ld90` for the LD90, and `gui:=false` for a headless server. The
launch starts a simple empty world, publishes the robot description and `/tf`,
and bridges Gazebo `/clock` to ROS. It sets the model static; no `/cmd_vel`,
odometry, laser scans, Nav2, or hardware status is simulated. Run one preview per
Gazebo world. Stop it with Ctrl+C.

For a warehouse environment, use:

```bash
source install/setup.bash
ros2 launch omron_amr_simulation preview.launch.py model:=ld250 world:=warehouse
```

The local warehouse layout is approximately 28 by 21 meters and adapts assets
from [Open Robotics' industrial-warehouse world](https://app.gazebosim.org/OpenRobotics/fuel/worlds/industrial-warehouse),
available on Gazebo Fuel under CC BY 4.0. Gazebo downloads and caches the Fuel
models on first use, which requires network access and can take time.
The default world remains `empty`. Set `x`, `y`, `z` (meters) and `yaw` (radians)
to choose a spawn pose; all default to zero.

The warehouse also has three open racks near `(-2.7, 0)` with 20 dynamic
120 mm boxes: four each in blue, red, green, black, and brown. Each weighs
0.35 kg and is an independent collision object. The existing Fuel shelf
contents remain part of their static shelf meshes.

Box positions are fixed by default. Set `randomize_boxes:=true` to distribute
them randomly across the racks at startup. Add `box_seed:=42` (or another seed)
to reproduce a randomized layout; the seed has no effect when randomization is
disabled.

### Manipulation objects

For a simple grasp-testing environment, launch the separate world:

```bash
ros2 launch omron_amr_simulation preview.launch.py model:=ld250 world:=grasp_test
```

The grasp-test world includes five worktables at `x=1.3`, with `y` positions
from -4 to 4 meters in 2 meter steps. Each table holds five dynamic objects:
a cube, cylinder, sphere, rectangular block, and bottle. They have collision
geometry and respond to gravity and contact; they are not attached to the
tables. Their dimensions and masses are example values, not verified gripper
specifications.

Object positions are fixed by default. Use `randomize_objects:=true` to shuffle
the five objects on each table at startup, and optionally `object_seed:=42` to
reproduce the layout. The seed has no effect when randomization is disabled.

The reusable models are in `omron_amr_simulation/models`. Add more instances
to a world SDF using the `model://manipulation_*` models, unique names, and
desired poses. The preview
adds that model directory to Gazebo's resource path. The default empty world
contains no objects; the grasp-test tables and their objects are separate
from the warehouse rack.

The robot remains static in this preview, so it cannot yet drive, close its
gripper, or pick up the objects. Grasping requires robot and gripper dynamics,
controllers, and validated contact behavior; vacuum gripping also needs a
suction simulation mechanism.

## Reuse and remaining work

- The existing `amr_description` Xacros provide the chassis geometry and, for
  LD250, laser frames. They contain no driven wheel joints or sensors. Their
  4.07 kg body masses and inertias need validation; LD90's visual and collision
  meshes also have different vertical origins. Do not treat them as physical
  specifications.
- Add measured wheel geometry, joints, masses, contact parameters and a drive
  system, plus Gazebo lidar sensors. Bridge or adapt the ROS interfaces used by
  the LD250 configuration: `cmd_vel`, `odom`, `odom -> base_link`, `scan`, and
  `scan_low`. Validate frame conventions, limits, watchdog behavior and timing
  before using Nav2 with `use_sim_time`.
- `amr_core` interfaces use libaria to connect to the Omron robot server and
  request hardware data; Gazebo does not implement that network protocol. They
  cannot directly drive this Gazebo world. A simulated ROS-facing adapter can
  preserve their topic contracts, or a separately implemented protocol emulator
  could exercise the real driver. Neither approach currently provides the full
  status, fault, docking, and other hardware services.
- Do not launch the existing hardware bringup against a reachable robot for a
  simulation test. The preview intentionally avoids `amr_core` entirely.

The package maintainer and license fields are placeholders to be decided before
publishing the repository.