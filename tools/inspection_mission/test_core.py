import copy
from dataclasses import asdict
from pathlib import Path
import tempfile
import unittest

from .core import Mission, Point, Policy, Pose, State, TaskSpec
from .io import durable_json, dwell_result, load_mission


class MissionTest(unittest.TestCase):
    def setUp(self):
        self.m = Mission([Point(f"P{i}", Pose(i, 0, 0, 0)) for i in range(1, 5)], "test-map")
        self.now = 0.0
        self.home = {"x": 10., "y": -3., "z": .1, "yaw": 2.}
        self.m.start(self.now)
        self.effect = self.m.drain_effects()[-1]

    def reply(self, event, data=None, token=None):
        self.now += .01
        accepted = self.m.handle(event, self.effect.token if token is None else token, self.now, data)
        effects = self.m.drain_effects()
        if effects:
            self.effect = effects[-1]
        return accepted

    def precheck(self):
        self.reply("precheck_ok", {"all_checks_ok": True, "map_id": "test-map", "home": self.home})

    def at_point(self):
        self.reply("nav_succeeded")
        self.reply("stopped", {"fresh_stationary_at_target": True})

    def complete_task(self):
        self.reply("inspection_succeeded", dwell_result(self.effect, 3.1))
        self.reply("committed", {"key": self.m.mission_id + "/" + self.m.point_id, "durable": True})

    def quiesce(self):
        self.reply("quiesced", {"workers_terminal": True, "fresh_stationary": True})

    def tick_deadline(self):
        self.now = self.m.deadline + .001
        self.m.tick(self.now)
        effects = self.m.drain_effects()
        if effects:
            self.effect = effects[-1]

    def test_settling_has_time_for_buffer_but_cannot_wait_forever(self):
        self.precheck()
        self.reply("nav_succeeded")
        self.now += 8.1
        self.m.tick(self.now)
        self.assertEqual(self.m.state, State.VERIFY_STOP)
        self.tick_deadline()
        self.assertEqual(self.m.state, State.FAILED)
        self.assertEqual(self.m.reason, "verify_stop_timeout")
        self.assertFalse(self.m.records)

    def test_order_and_return_to_captured_start(self):
        self.precheck()
        goals = []
        for point_id in ("P1", "P2", "P3", "P4"):
            self.assertEqual(self.effect.point_id, point_id)
            goals.append(self.effect.point_id)
            self.at_point()
            self.complete_task()
        self.assertEqual(self.m.state, State.RETURNING)
        self.assertEqual(self.effect.point_id, "HOME")
        self.assertEqual(self.effect.payload["pose"], self.home)
        self.reply("nav_succeeded")
        self.reply("stopped", {"fresh_stationary_at_target": True})
        self.assertEqual(self.m.state, State.FINISHING)
        self.reply("finished", {"durable": True})
        self.assertEqual(self.m.state, State.COMPLETED)
        self.assertEqual([r["point_id"] for r in self.m.records], goals)

    def test_navigation_alone_cannot_advance(self):
        self.precheck()
        self.reply("nav_succeeded")
        self.assertEqual(self.m.index, 0)
        self.assertEqual(self.m.state, State.VERIFY_STOP)
        self.assertFalse(self.reply("stopped", {"fresh_stationary_at_target": False}))
        self.assertEqual(self.m.index, 0)

    def test_early_inspection_result_is_ignored(self):
        self.precheck()
        self.assertFalse(self.reply("inspection_succeeded", {}))
        self.assertEqual(self.m.state, State.NAVIGATING)

    def test_old_and_duplicate_callbacks_do_not_advance(self):
        self.precheck()
        nav_token = self.effect.token
        self.at_point()
        self.assertFalse(self.reply("nav_succeeded", token=nav_token))
        result = dwell_result(self.effect, 3.1)
        inspect_token = self.effect.token
        self.reply("inspection_succeeded", result)
        self.assertFalse(self.reply("inspection_succeeded", result, token=inspect_token))
        self.assertEqual(self.m.state, State.COMMITTING)
        self.assertEqual(self.m.index, 0)

    def test_commit_required_and_payload_is_copied(self):
        self.precheck()
        self.at_point()
        result = dwell_result(self.effect, 3.1)
        self.reply("inspection_succeeded", result)
        result["success"] = False
        self.assertTrue(self.m.pending_record["success"])
        self.assertFalse(self.reply("committed", {"key": "wrong", "durable": True}))
        self.assertEqual(self.m.index, 0)

    def test_task_failure_does_not_skip_and_has_bounded_retries(self):
        self.precheck()
        self.at_point()
        for attempt in range(3):
            self.reply("failed", {"reason": "test_failure"})
            self.assertEqual(self.m.state, State.CANCELING)
            self.quiesce()
            if attempt < 2:
                self.tick_deadline()
                self.reply("stopped", {"fresh_stationary_at_target": True})
        self.assertEqual(self.m.state, State.FAILED)
        self.assertEqual(self.m.index, 0)
        self.assertEqual(self.m.inspect_attempts, 3)

    def test_timeout_rejects_late_success_and_waits_for_cancel(self):
        self.precheck()
        token = self.effect.token
        self.tick_deadline()
        self.assertFalse(self.reply("nav_succeeded", token=token))
        self.assertFalse(self.reply("quiesced", {"workers_terminal": False, "fresh_stationary": True}))
        self.assertEqual(self.m.state, State.CANCELING)
        self.quiesce()
        self.tick_deadline()
        self.assertNotEqual(self.effect.token, token)
        self.assertEqual(self.m.nav_attempts, 2)
        self.assertEqual(self.m.index, 0)

    def test_missing_cancel_ack_fails_without_another_goal(self):
        self.precheck()
        self.tick_deadline()
        self.tick_deadline()
        self.assertEqual(self.m.state, State.FAILED)
        self.assertEqual(self.m.nav_attempts, 1)

    def test_pause_resume_keeps_original_home(self):
        self.precheck()
        self.now += .1
        self.m.pause(self.now, "localization_lost")
        self.effect = self.m.drain_effects()[-1]
        self.quiesce()
        self.assertEqual(self.m.state, State.PAUSED)
        self.now += .1
        self.m.resume(self.now)
        self.effect = self.m.drain_effects()[-1]
        self.home["x"] = 999.
        self.precheck()
        self.assertEqual(self.m.home.x, 10.)
        self.assertEqual(self.m.index, 0)

    def test_storage_failure_never_advances(self):
        self.precheck()
        self.at_point()
        self.reply("inspection_succeeded", dwell_result(self.effect, 3.1))
        self.reply("failed", {"reason": "disk_full"})
        self.assertEqual(self.m.state, State.FAILED)
        self.assertEqual(self.m.index, 0)

    def test_wrong_point_and_fake_recognition_rejected(self):
        self.precheck()
        self.at_point()
        result = dwell_result(self.effect, 3.1)
        result["point_id"] = "P4"
        self.reply("inspection_succeeded", result)
        self.assertEqual(self.m.state, State.CANCELING)
        self.assertEqual(self.m.index, 0)

    def test_external_task_needs_valid_data_and_result_uri(self):
        self.m = Mission([Point(f"P{i}", Pose(i, 0, 0, 0), TaskSpec("external")) for i in range(1, 5)], "test-map")
        self.m.start(0.)
        self.effect = self.m.drain_effects()[-1]
        self.precheck()
        self.at_point()
        result = {"mission_id": self.m.mission_id, "point_id": "P1", "request_id": self.effect.token,
                  "map_id": "test-map", "task_kind": "external", "success": True}
        self.reply("inspection_succeeded", result)
        self.assertEqual(self.m.state, State.CANCELING)

    def test_duplicate_start_and_bad_clock(self):
        with self.assertRaises(RuntimeError):
            self.m.start(.1)
        with self.assertRaises(ValueError):
            self.m.tick(.05)

    def test_global_deadline_includes_pause(self):
        self.precheck()
        self.m.pause(.1)
        self.m.tick(self.m.mission_deadline)
        self.assertEqual(self.m.state, State.FAILED)

    def test_return_failure_is_not_completion(self):
        self.precheck()
        for _ in range(4):
            self.at_point()
            self.complete_task()
        self.reply("failed", {"reason": "home_blocked"})
        self.assertEqual(self.m.state, State.CANCELING)
        self.assertEqual(len(self.m.records), 4)

    def test_real_configuration_and_atomic_storage(self):
        mission, route = load_mission(str(Path(__file__).resolve().parents[2] / "routes/nv_zup_ccw90/inspection_mission.yaml"))
        self.assertEqual(mission.points[0].pose.x, 1.532753825)
        self.assertEqual(mission.points[-1].task.kind, "dwell")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/"record.json"
            durable_json(path, {"point": "P1"})
            durable_json(path, {"point": "P1"})
            with self.assertRaises(ValueError):
                durable_json(path, {"point": "P2"})


if __name__ == "__main__":
    unittest.main()
