# Dependencies

The tracked packages target ROS 2 Humble and `ament_cmake`. ROS 1 is not a
supported build target for this source tree.

The release vendors the small FAST-LIO source tree at commit
`a4743b095409588842a5b30ddfa27e29d2f99164`, including the pinned `ikd-Tree`
source as ordinary files, so a clone does not need a submodule checkout.
Import the optional Unitree repositories before a physical-robot build:

```bash
vcs import ros2_ws/src < dependencies/ros2.repos
```

Install system dependencies before the build with `rosdep install ...` (or
run `RUN_ROSDEP=1 scripts/build.sh`). The default build script does not invoke
`sudo` implicitly.

The FAST-LIO `ikd-Tree` source is required. Unitree ROS 2 uses the patch in
`dependencies/patches/unitree_ros2.patch`; SDK2 is needed only for
`enable_motion:=true`.

The Livox ROS 2 driver currently requires the Livox SDK2 shared library and
headers. The release retains the SDK2 source under the driver package; the
build script must install it into a configurable prefix before building the
ROS package. No `/usr/local` binary is part of this repository.

For the offline/simulation profile, build and run with motion and hardware
drivers disabled. The real-robot profile additionally requires the Unitree
ROS 2 overlay, SDK2 libraries, a MID360, and the robot network interface.
