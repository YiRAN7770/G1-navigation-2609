# G1 FAST-LIO/FAR navigation

> **Near-goal control branch:** `feature/near-goal-control-20261020` saves the
> current navigation code: reverse within 0.50 m, align within 0.20 m, then
> translate while holding goal yaw. Success still requires position ≤0.10 m
> and yaw error ≤20°. Use the independent single-lap `tools/inspection_route.py`,
> not the historical inspection executor. See [saved version and commands](docs/save_near_goal_20261020.md)
> and [controller details](docs/near_goal_control.md). Isolated tests passed;
> this is not a real-robot acceptance result. The old baseline remains on
> `restore/pre-final-pose-20260929`.

This repository contains the ROS 2 Humble workspace used by the current G1
navigation chain: FAST-LIO, prior-map localization, FAR, the local planner,
and the `site_04_nav_zup_ccw90` route and prior map.

The published tree is intentionally a source release. It includes the ROS
packages, launch/configuration files, the vendored FAST-LIO `ikd-Tree` source,
the Livox SDK2 source, the route, and the map. It excludes bags, run logs,
compiled workspaces, SDK libraries, and historical experimental entrypoints.

The main launch file is:

```text
ros2_ws/src/base_autonomy/vehicle_simulator/launch/system_real_robot_with_fastlio_far.launch.py
```

Build in a ROS 2 Humble shell with:

```bash
bash scripts/build.sh
```

The default launch profile keeps `enable_motion:=false`. Physical motion
requires the external Unitree overlay and the environment variables described
in [docs/dependencies.md](docs/dependencies.md) and
[docs/runtime_paths.md](docs/runtime_paths.md).

For an offline check of the current patrol script, run:

```bash
source /opt/ros/humble/setup.bash
python3 tools/inspection_route.py show
python3 -m unittest discover -s tools -p test_inspection_route.py -v
```

The current ROS integration suites use synthetic localhost-only DDS and do
not start SDK2 or a robot. See [saved-version tests](docs/save_near_goal_20261020.md)
for the required `ROS_LOCALHOST_ONLY=1` and nonzero `ROS_DOMAIN_ID` safety gate.

Verify the published map assets with:

```bash
sha256sum -c routes/nv_zup_ccw90/map.sha256
```

The map alignment record remains explicit in
`maps/site_04_nav_zup_ccw90/conversion.json`; field alignment still requires a
separate acceptance check.
