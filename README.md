# G1 FAST-LIO/FAR navigation

> **Historical navigation branch:** `restore/pre-final-pose-20260929` restores
> the 12 backed-up navigation source/configuration files from before the
> September 29 reverse/final-pose repair. The known behind-waypoint stop is
> intentionally present. The later inspection executor is retained but is not
> validated against this older status interface. See
> [restoration scope and commands](docs/restore_pre_final_pose_20260929.md)
> before using the mainline instructions below. This is not a full-workspace
> historical snapshot or a passing robot-navigation release.

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

For an offline deterministic check, run:

```bash
source /opt/ros/humble/setup.bash
python3 -m tools.inspection_mission.demo --config routes/nv_zup_ccw90/inspection_mission.yaml
python3 -m unittest tools.inspection_mission.test_core \
  tools.inspection_mission.test_progress \
  tools.inspection_mission.test_settling \
  tools.inspection_mission.test_stationary
```

The optional ROS integration suite uses synthetic localhost-only DDS and does
not start SDK2 or a robot. See `tools/inspection_mission/test_ros.py` for the
required `ROS_LOCALHOST_ONLY=1` and nonzero `ROS_DOMAIN_ID` safety gate.

Verify the published map assets with:

```bash
sha256sum -c routes/nv_zup_ccw90/map.sha256
```

The map alignment record remains explicit in
`maps/site_04_nav_zup_ccw90/conversion.json`; field alignment still requires a
separate acceptance check.
