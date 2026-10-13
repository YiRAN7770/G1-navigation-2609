#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
G1_ROOT="$(cd "${SCRIPT_DIR}/.." > /dev/null && pwd)"
UNITREE_SETUP="${UNITREE_SETUP:-}"
FASTLIO_SETUP="${G1_FASTLIO_SETUP:-${G1_ROOT}/ros2_ws/install/setup.bash}"
G1_CONTROLLER_SETUP="${G1_CONTROLLER_SETUP:-${G1_ROOT}/ros2_ws/install/setup.bash}"
START_RVIZ="${ENABLE_RVIZ:-auto}"
PLANNER_MODE="far"
MOTION_ENABLED=0
ENV_READY=0
LAUNCH_ARGS=(operation:=localization planner_mode:=far enable_motion:=false start_mid360_driver:=false start_fastlio:=false start_joy:=false start_visualization_tools:=false lidar_inverted:=true initial_heading_search:=true route_planner_config:=site04_zup "map:=${G1_ROOT}/maps/site_04_nav_zup_ccw90/prior.pcd")
LAUNCH_PID=""
RVIZ_PID=""
LAUNCH_PGID=""
RVIZ_PGID=""
_CLEANED_UP=0
STARTUP_WAIT_PID=""
SESSION_HELPER="${SCRIPT_DIR}/../tools/fastlio_far_session.py"

# Recovery must remain available even while the legacy wrapper lock is held.
if [[ "${1:-}" == "--status" || "${1:-}" == "--stop" ]]; then
  [[ "$#" -eq 1 ]] || { echo "Use --status or --stop without launch arguments." >&2; exit 2; }
  exec python3 "${SESSION_HELPER}" "$1"
fi

# Keep the inode: unlinking a flock file allows competing locks on two inodes.
LOCK_FILE="/tmp/g1_fastlio_far_wrapper.lock"
exec {WORKFLOW_LOCK_FD}>"${LOCK_FILE}"
if ! flock -n "${WORKFLOW_LOCK_FD}"; then
  echo "FAST-LIO/FAR workflow lock is held; a wrapper or inherited child session is still active." >&2
  python3 "${SESSION_HELPER}" --status >&2
  echo "Inspect: bash \"${BASH_SOURCE[0]}\" --status" >&2
  echo "Stop the existing session first: bash \"${BASH_SOURCE[0]}\" --stop" >&2
  exit 2
fi

terminate_group() {
  local pgid="$1"
  [[ -n "${pgid}" ]] || return 0
  # A shutdown can arrive between fork and the child's setsid(). Let the
  # session form before signalling it; if setup failed, stop only that child.
  for _ in {1..10}; do
    kill -0 "${pgid}" 2>/dev/null || break
    kill -0 -- "-${pgid}" 2>/dev/null && break
    sleep 0.1
  done
  if ! kill -0 -- "-${pgid}" 2>/dev/null; then
    kill -TERM "${pgid}" 2>/dev/null || true
    return 0
  fi
  if kill -0 -- "-${pgid}" 2>/dev/null; then
    # Signal the launch/session leader first.  ros2 launch then shuts down its
    # children exactly once.  Broadcasting SIGINT to the whole group made some
    # nodes receive both our signal and launch's signal and abort while rclcpp
    # was already shutting down.
    kill -INT "${pgid}" 2>/dev/null || true
    for _ in {1..80}; do
      kill -0 -- "-${pgid}" 2>/dev/null || return 0
      sleep 0.1
    done
    kill -TERM -- "-${pgid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 -- "-${pgid}" 2>/dev/null || return 0
      sleep 0.1
    done
    kill -KILL -- "-${pgid}" 2>/dev/null || true
  fi
}

cleanup() {
  if [[ "${_CLEANED_UP}" -eq 1 ]]; then
    return
  fi
  _CLEANED_UP=1
  if [[ -n "${STARTUP_WAIT_PID}" ]]; then
    kill "${STARTUP_WAIT_PID}" 2>/dev/null || true
    wait "${STARTUP_WAIT_PID}" 2>/dev/null || true
  fi
  if [[ "${MOTION_ENABLED}" -eq 1 && "${ENV_READY}" -eq 1 && -n "${LAUNCH_PID}" ]] &&
     kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    timeout 2 ros2 topic pub --once /fastlio_far/stop std_msgs/msg/Bool \
      "{data: true}" >/dev/null 2>&1 || true
  fi
  if [[ -n "${RVIZ_PGID}" ]]; then
    terminate_group "${RVIZ_PGID}"
  fi
  if [[ -n "${LAUNCH_PGID}" ]]; then
    terminate_group "${LAUNCH_PGID}"
  fi

}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

start_session() {
  # The wrapper alone owns its flock; launch has a separate lock of its own.
  python3 "${SESSION_HELPER}" --exec-session "$@" {WORKFLOW_LOCK_FD}>&- &
  SESSION_PID=$!
}

for arg in "$@"; do
  case "$arg" in
    --no-rviz)
      START_RVIZ=0
      ;;
    --rviz)
      START_RVIZ=1
      ;;
    planner_mode:=*)
      PLANNER_MODE="${arg#planner_mode:=}"
      LAUNCH_ARGS+=("$arg")
      ;;
    enable_motion:=*)
      MOTION_ENABLED=0
      [[ "${arg#enable_motion:=}" == "true" ]] && MOTION_ENABLED=1
      LAUNCH_ARGS+=("$arg")
      ;;
    *)
      LAUNCH_ARGS+=("$arg")
      ;;
  esac
done

if [[ ! -f "${FASTLIO_SETUP}" ]]; then
  echo "Missing ROS2 overlay: ${FASTLIO_SETUP}. Run scripts/build.sh first." >&2
  exit 1
fi

if [[ -n "${UNITREE_SETUP}" && -f "${UNITREE_SETUP}" ]]; then
  source "${UNITREE_SETUP}"
fi
source "${FASTLIO_SETUP}"
if [[ "${G1_CONTROLLER_SETUP}" != "${FASTLIO_SETUP}" && -f "${G1_CONTROLLER_SETUP}" ]]; then
  source "${G1_CONTROLLER_SETUP}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/g1_fastlio_far_ros_log}"
mkdir -p "${ROS_LOG_DIR}"
ENV_READY=1

# All of these nodes share un-namespaced map and velocity topics.  Starting a
# second workflow would mix maps, TF and commands, so reject both a live second
# wrapper and orphaned nodes left by a previously interrupted terminal.
if ! python3 "${SESSION_HELPER}" --check; then
  echo "Existing workflow detected; inspect with --status and stop it with --stop before restarting." >&2
  exit 2
fi

if [[ "$START_RVIZ" == "auto" ]]; then
  if [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]; then
    START_RVIZ=1
  else
    START_RVIZ=0
  fi
fi

if [[ "${MOTION_ENABLED}" -eq 1 ]]; then
  if [[ ! -f "${G1_CONTROLLER_SETUP}" ]]; then
    echo "Cannot enable motion: missing ${G1_CONTROLLER_SETUP}" >&2
    exit 1
  fi
  if pgrep -af '[c]md_vel_to_g1|[u]nitree_control.py' >/dev/null; then
    echo "Cannot enable motion: another G1 motion backend is already running:" >&2
    pgrep -af '[c]md_vel_to_g1|[u]nitree_control.py' >&2 || true
    exit 1
  fi
  echo "WARNING: G1 MOTION ENABLED. Keep an operator at the emergency stop."
fi

echo "This launch starts sensing, FAST-LIO and planning. Motion is opt-in: enable_motion:=true"
start_session ros2 launch vehicle_simulator system_real_robot_with_fastlio_far.launch.py "${LAUNCH_ARGS[@]}"
LAUNCH_PID=${SESSION_PID}
LAUNCH_PGID="${LAUNCH_PID}"

sleep 7 {WORKFLOW_LOCK_FD}>&- &
STARTUP_WAIT_PID=$!
wait "${STARTUP_WAIT_PID}"
STARTUP_WAIT_PID=""
if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
  wait "${LAUNCH_PID}"
  exit $?
fi

GRAPH_VGH="${G1_GRAPH_VGH:-${G1_ROOT}/maps/site_04_nav_zup_ccw90/site_04_compact_trajectory.vgh}"
timeout 5 ros2 topic pub --once /read_file_dir std_msgs/msg/String \
  "{data: '${GRAPH_VGH}'}" >/dev/null 2>&1 || \
  echo "Warning: FAR graph was not accepted yet; publish ${GRAPH_VGH} to /read_file_dir." >&2

if [[ "${START_RVIZ}" == "1" ]]; then
  RVIZ_CONFIG="${G1_ROOT}/ros2_ws/src/route_planner/far_planner/rviz/fastlio.rviz"
  start_session ros2 run rviz2 rviz2 -d "${RVIZ_CONFIG}"
  RVIZ_PID=${SESSION_PID}
  RVIZ_PGID="${RVIZ_PID}"
  # Stop the sibling session when either launch or RViz exits.
  wait -n "${LAUNCH_PID}" "${RVIZ_PID}"
else
  wait "${LAUNCH_PID}"
fi
