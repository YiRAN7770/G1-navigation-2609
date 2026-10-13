"""Pure offline deterministic demonstration: synthetic poses, no ROS imports."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path

from .core import State
from .io import durable_json, dwell_result, load_mission


DEFAULT_CONFIG = str(Path(__file__).resolve().parents[2] / "routes/nv_zup_ccw90/inspection_mission.yaml")


def run(config=DEFAULT_CONFIG, scenario="success", output=None):
    mission, route = load_mission(config)
    now, injected = 0.0, False
    home = {"x": 0.25, "y": -0.5, "z": 0.1, "yaw": 0.2}
    mission.start(now)
    for _ in range(250):
        for effect in mission.drain_effects():
            print(f"{now:6.2f} {mission.state.value:12} {effect.point_id:4} {effect.kind}")
            now += .05
            reply, data = None, {}
            if effect.kind == "precheck":
                reply, data = "precheck_ok", {"all_checks_ok": True, "map_id": route["map_id"], "home": home}
            elif effect.kind == "navigate":
                if scenario == "nav_timeout" and effect.point_id == "P2" and not injected:
                    injected = True
                    now = mission.deadline + .01
                    mission.tick(now)
                else:
                    reply = "nav_succeeded"
            elif effect.kind == "hold_and_verify":
                reply, data = "stopped", {"fresh_stationary_at_target": True}
            elif effect.kind == "inspect":
                if scenario == "task_failure" and effect.point_id == "P2":
                    reply, data = "failed", {"reason": "synthetic_task_failure"}
                elif scenario == "stale_result" and effect.point_id == "P2" and not injected:
                    injected = True
                    assert not mission.handle("inspection_succeeded", "old-request", now, {})
                    now += .01
                    reply, data = "inspection_succeeded", dwell_result(effect, 3.0)
                else:
                    reply, data = "inspection_succeeded", dwell_result(effect, 3.0)
            elif effect.kind == "commit":
                if output:
                    durable_json(Path(output) / mission.mission_id / (effect.point_id + ".json"),
                                 {"simulation": True, **effect.payload["record"]})
                reply, data = "committed", {"key": effect.payload["key"], "durable": True}
            elif effect.kind == "cancel_and_hold":
                reply, data = "quiesced", {"workers_terminal": True, "fresh_stationary": True}
            elif effect.kind == "finish_record":
                reply, data = "finished", {"durable": True}
            if reply:
                mission.handle(reply, effect.token, now, data)
        if mission.state in mission.TERMINAL:
            break
        if mission.state == State.RETRY_WAIT:
            now = mission.deadline + .01
            mission.tick(now)
    report = {"simulation": True, "mission_id": mission.mission_id,
              "state": mission.state.value, "reason": mission.reason,
              "home": asdict(mission.home), "points": [r["point_id"] for r in mission.records],
              "recognition_status": "not_configured", "history": mission.history}
    if output:
        durable_json(Path(output) / mission.mission_id / "summary.json", report)
    print(json.dumps({k: v for k, v in report.items() if k != "history"}, ensure_ascii=False, indent=2))
    return mission


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--scenario", choices=["success", "nav_timeout", "task_failure", "stale_result"], default="success")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    run(args.config, args.scenario, args.output)


if __name__ == "__main__":
    main()
