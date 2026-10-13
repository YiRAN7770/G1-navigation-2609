#!/usr/bin/env python3
"""Inspect/recover only the local FAST-LIO/FAR workflow; never unlink locks."""
import fcntl
import os
from pathlib import Path
import signal
import sys
import time

WRAPPER = 'run_fastlio_far.sh'
LAUNCH = 'system_real_robot_with_fastlio_far.launch.py'
ROOT = Path(__file__).resolve().parent.parent
RVIZ_CONFIGS = {
    ROOT / 'ros2_ws/src/route_planner/far_planner/rviz/default.rviz',
    ROOT / 'ros2_ws/src/route_planner/far_planner/rviz/fastlio.rviz',
}
LOCKS = [Path('/tmp/g1_fastlio_far_' + name + '.lock') for name in ('wrapper', 'launch')]
NODE_NAMES = {
    'fastlio_mapping', 'mapping_map_to_camera_init', 'mapping_frame_adapter',
    'far_planner', 'visualizationTools', 'graph_decoder', 'cmd_vel_to_g1',
    'cmd_vel_to_g1_persistent',
    'localPlanner', 'pathFollower', 'terrainAnalysis', 'terrainAnalysisExt',
    'sensorScanGeneration', 'prior_map_localizer', 'static_transform_publisher',
    'g1_loco_client', 'unitree_control.py', 'livox_ros_driver2_node',
    'fastlio_far_motion_gate',
}


def process(pid):
    try:
        p = Path('/proc') / str(pid)
        if p.stat().st_uid != os.getuid():
            return None
        fields = (p / 'stat').read_text().rsplit(') ', 1)[1].split()
        if fields[0] == 'Z':
            return None
        argv = (p / 'cmdline').read_bytes().decode(errors='replace').rstrip('\0').split('\0')
        env = dict(item.split(b'=', 1) for item in (p / 'environ').read_bytes().split(b'\0') if b'=' in item)
        return dict(pid=pid, ppid=int(fields[1]), pgid=int(fields[2]), start=fields[19],
                    argv=argv, domain=env.get(b'ROS_DOMAIN_ID', b'0').decode(errors='replace'))
    except (OSError, ValueError, IndexError):
        return None


def processes():
    return [p for entry in Path('/proc').iterdir() if entry.name.isdigit()
            if (p := process(int(entry.name))) is not None]


def kind(p):
    # Match argument positions, not arbitrary shell command text.
    a = p['argv']
    if not a:
        return None
    executable = Path(a[0]).name
    script = Path(a[1]).name if len(a) > 1 else ''
    if executable == WRAPPER or (executable in ('bash', 'sh') and script == WRAPPER):
        if '--status' not in a and '--stop' not in a:
            return 'wrapper'
    ros = a[1:] if executable.startswith('python') else a
    if len(ros) >= 4 and Path(ros[0]).name == 'ros2' and ros[1:4] == ['launch', 'vehicle_simulator', LAUNCH]:
        return 'launch'
    if p['domain'] != '0':
        return None
    is_rviz = executable == 'rviz2' or (executable.startswith('python') and script == 'rviz2')
    is_rviz = is_rviz or (len(ros) >= 4 and Path(ros[0]).name == 'ros2' and ros[1:4] == ['run', 'rviz2', 'rviz2'])
    if is_rviz and '-d' in a:
        index = a.index('-d') + 1
        if index < len(a) and Path(a[index]).resolve() in RVIZ_CONFIGS:
            return 'rviz'
    if executable in NODE_NAMES or (executable.startswith('python') and script in NODE_NAMES):
        return 'node'
    if len(ros) >= 3 and Path(ros[0]).name == 'ros2' and ros[1:3] == ['bag', 'play']:
        return 'bag player'
    return None


def workflows():
    all_p = processes()
    by_pid = {p['pid']: p for p in all_p}
    ancestors = set()
    pid = os.getpid()
    while pid in by_pid and pid not in ancestors:
        ancestors.add(pid)
        pid = by_pid[pid]['ppid']
    return [(p, k) for p in all_p if p['pid'] not in ancestors if (k := kind(p))]


def locks():
    result = []
    for path in LOCKS:
        try:
            # An absent file is free; status must not create a new lock inode.
            with path.open('r') as f:
                try:
                    fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    held = False
                except BlockingIOError:
                    held = True
                result.append((path, held))
        except FileNotFoundError:
            result.append((path, False))
    return result


def status():
    found = workflows()
    for p, k in found:
        print(f"{k}: PID={p['pid']} PGID={p['pgid']} " + ' '.join(p['argv']), flush=True)
    if not found:
        print('No FAST-LIO/FAR workflow processes detected.', flush=True)
    for path, held in locks():
        print(f'{path}: {"HELD" if held else "FREE"}', flush=True)
    return found


def same(p):
    now = process(p['pid'])
    return now is not None and now['start'] == p['start']


def notify(p, sig):
    if same(p):
        try:
            os.kill(p['pid'], sig)
        except ProcessLookupError:
            pass


def wait_gone(targets, seconds):
    deadline = time.monotonic() + seconds
    while any(same(p) for p in targets) and time.monotonic() < deadline:
        time.sleep(.1)
    return not any(same(p) for p in targets)


def stop():
    found = workflows()
    owners = [(p, k) for p, k in found if k in ('wrapper', 'launch', 'rviz')]
    if not owners:
        status()
        if found or any(held for _, held in locks()):
            print('No identifiable wrapper/launch/workflow RViz owns these leftovers; inspect PIDs manually.', file=sys.stderr)
            return 2
        return 0
    # Capture exact identities before signalling. Escalation never targets a new session.
    snapshot = processes()
    owner_ids = {p['pid'] for p, _ in owners}
    groups = {p['pgid'] for p, k in owners if k in ('launch', 'rviz') and p['pgid'] == p['pid']}
    selected = set(owner_ids)
    for _ in range(len(snapshot)):
        added = {p['pid'] for p in snapshot if p['ppid'] in selected or p['pgid'] in groups}
        if added <= selected:
            break
        selected.update(added)
    targets = [p for p in snapshot if p['pid'] in selected]
    wrappers = [p for p, k in owners if k == 'wrapper']
    for p in wrappers:
        print(f"Stopping wrapper PID {p['pid']}", flush=True)
        notify(p, signal.SIGTERM)
    if wrappers:
        wait_gone(targets, 24)
    for p, k in owners:
        if k in ('launch', 'rviz'):
            print(f"Stopping {k} PID {p['pid']}", flush=True)
            notify(p, signal.SIGINT)
    if not wait_gone(targets, 8):
        # Legacy background launches may have inherited SIGINT=SIG_IGN.
        for p in targets:
            if p['pid'] not in owner_ids:
                notify(p, signal.SIGINT)
        wait_gone(targets, 5)
        for p in targets:
            notify(p, signal.SIGTERM)
        if not wait_gone(targets, 5):
            for p in targets:
                notify(p, signal.SIGKILL)
            wait_gone(targets, 2)
    remaining = status()
    return 2 if remaining or any(held for _, held in locks()) else 0


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else '--status'
    if mode == '--exec-session':
        # Background bash commands inherit ignored SIGINT/SIGQUIT. Reset before
        # ros2 installs handlers, and give the child its own process group.
        os.setsid()
        for sig in (signal.SIGINT, signal.SIGQUIT, signal.SIGHUP, signal.SIGTERM):
            signal.signal(sig, signal.SIG_DFL)
        os.execvp(sys.argv[2], sys.argv[2:])
    elif mode == '--status':
        status()
        return 0
    elif mode == '--stop':
        return stop()
    elif mode == '--check':
        found = workflows()
        if found or any(held for path, held in locks() if path == LOCKS[1]):
            status()
            return 2
        return 0
    raise SystemExit('Usage: --status | --stop | --check | --exec-session COMMAND ...')


if __name__ == '__main__':
    sys.exit(main())
