"""Configuration and durable, idempotent JSON records."""
from dataclasses import asdict
import json
import os
from pathlib import Path
import tempfile
import yaml

from .core import Mission, Point, Policy, Pose, TaskSpec


def load_mission(path):
    config = yaml.safe_load(Path(path).read_text())
    route_path = Path(config["route"])
    if not route_path.is_absolute():
        route_path = Path(path).resolve().parent / route_path
    route = yaml.safe_load(route_path.read_text())
    # Resolve map assets relative to route.yaml, never relative to the caller's
    # current working directory. This keeps a cloned workspace portable.
    for key in ("prior_pcd", "graph_vgh"):
        asset = Path(route[key])
        if not asset.is_absolute():
            asset = route_path.parent / asset
        route[key] = str(asset.resolve())
    if (config["map_id"] != route["map_id"] or route["frame_id"] != "map"
            or config["sequence"] != ["P1", "P2", "P3", "P4"]
            or config["home_mode"] != "capture_start_pose"
            or config["skip_failed_point"] is not False):
        raise ValueError("Unsupported or inconsistent mission/map configuration")
    if [p["id"] for p in route["points"]] != config["sequence"]:
        raise ValueError("Route points must uniquely match the mission sequence")
    points = [Point(p["id"], Pose(**p["position"], yaw=p["yaw"]),
                    TaskSpec(**config["tasks"][p["id"]])) for p in route["points"]]
    mission = Mission(points, route["map_id"], Policy(**config["policy"]))
    if any(p.task.dwell_seconds >= mission.policy.inspect_timeout for p in points):
        raise ValueError("Task timeout must exceed dwell duration")
    return mission, route


def durable_json(path, data):
    """Single writer. Identical repeat is allowed; conflicting overwrite fails."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = json.dumps(data, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False) + "\n"
    if path.exists():
        if path.read_text() != encoded:
            raise ValueError(f"Conflicting durable record: {path}")
        return
    fd, temporary = tempfile.mkstemp(prefix=".pending-", dir=path.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def dwell_result(effect, seconds):
    return {"mission_id": effect.mission_id, "point_id": effect.point_id,
            "request_id": effect.token, "map_id": effect.payload["map_id"],
            "task_kind": "dwell", "success": True, "stationary_seconds": seconds,
            "recognition_status": "not_configured"}
