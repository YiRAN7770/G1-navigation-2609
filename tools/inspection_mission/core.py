"""Single-owner, event-driven mission core. No ROS, network or motor writes.

Adapters execute Effects asynchronously and return matching tokens. Every
success event is a contract: adapters must check live health, source freshness,
goal identity and physical stop as documented in AUTO_INSPECTION_RUNBOOK.md.
Only one executor thread may call this object. Use monotonic seconds for `now`;
image/pose stamps remain ROS nanoseconds and must never be compared to `now`.
"""
from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Any
from uuid import uuid4


class State(str, Enum):
    IDLE = "IDLE"
    PRECHECK = "PRECHECK"
    NAVIGATING = "NAVIGATING"
    RETURNING = "RETURNING"
    VERIFY_STOP = "VERIFY_STOP"
    INSPECTING = "INSPECTING"
    COMMITTING = "COMMITTING"
    CANCELING = "CANCELING"
    RETRY_WAIT = "RETRY_WAIT"
    PAUSED = "PAUSED"
    FINISHING = "FINISHING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float

    def __post_init__(self):
        if not all(finite(v) for v in asdict(self).values()):
            raise ValueError("Pose must contain finite numbers")


@dataclass(frozen=True)
class TaskSpec:
    kind: str = "dwell"
    dwell_seconds: float = 3.0

    def __post_init__(self):
        if self.kind not in ("dwell", "external") or not finite(self.dwell_seconds) or self.dwell_seconds <= 0:
            raise ValueError("Invalid point task specification")


@dataclass(frozen=True)
class Point:
    point_id: str
    pose: Pose
    task: TaskSpec = TaskSpec()


@dataclass(frozen=True)
class Policy:
    precheck_timeout: float = 15.0
    nav_timeout: float = 120.0
    stop_timeout: float = 30.0
    settle_seconds: float = 5.0
    stationary_seconds: float = 3.0
    inspect_timeout: float = 30.0
    commit_timeout: float = 5.0
    cancel_timeout: float = 5.0
    retry_delay: float = 2.0
    mission_timeout: float = 1200.0
    nav_retries: int = 2
    inspect_retries: int = 2
    stopped_linear_speed: float = 0.05

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key.endswith("retries"):
                if type(value) is not int or value < 0:
                    raise ValueError(key)
            elif not finite(value) or value <= 0:
                raise ValueError(key)
        if self.stop_timeout <= self.settle_seconds + self.stationary_seconds:
            raise ValueError("stop_timeout must exceed settling plus stationary duration")


@dataclass(frozen=True)
class Effect:
    kind: str
    token: str
    mission_id: str
    point_id: str
    payload: dict[str, Any]


def validate_task_result(data, spec, *, mission_id, point_id, token, map_id):
    """Dwell is a navigation-only placeholder, never a recognition success.

    Future external workers must validate their actual recognition/evidence
    before declaring data_valid. Core enforces correlation, not model quality.
    """
    try:
        expected = {"mission_id": mission_id, "point_id": point_id,
                    "request_id": token, "map_id": map_id,
                    "task_kind": spec.kind}
        if any(data.get(k) != v for k, v in expected.items()):
            return False
        if data.get("success") is not True:
            return False
        if spec.kind == "dwell":
            return (data.get("recognition_status") == "not_configured"
                    and finite(data.get("stationary_seconds"))
                    and data["stationary_seconds"] >= spec.dwell_seconds)
        return (data.get("data_valid") is True and isinstance(data.get("result_uri"), str)
                and bool(data["result_uri"].strip()))
    except (KeyError, TypeError, ValueError):
        return False


class Mission:
    TERMINAL = (State.COMPLETED, State.FAILED)

    def __init__(self, points, map_id, policy=Policy()):
        self.points = tuple(points)
        ids = [p.point_id for p in self.points]
        if ids != ["P1", "P2", "P3", "P4"] or not map_id:
            raise ValueError("This mission requires P1 -> P2 -> P3 -> P4 and a map ID")
        self.map_id, self.policy = map_id, policy
        self.mission_id = uuid4().hex
        self.state, self.index = State.IDLE, 0
        self.home = None
        self.token, self.deadline = "", math.inf
        self.mission_deadline, self.last_now = math.inf, -math.inf
        self.nav_attempts, self.inspect_attempts = 0, 0
        self.effects, self.records, self.history = [], [], []
        self.reason, self.retry_stage, self.after_cancel = "", "", ""
        self.pending_record = None

    @property
    def point_id(self):
        return self.points[self.index].point_id if self.index < len(self.points) else "HOME"

    def drain_effects(self):
        out, self.effects = self.effects, []
        return out

    def _clock(self, now):
        if not finite(now) or now < self.last_now:
            raise ValueError("Use one monotonically increasing clock")
        self.last_now = now

    def _set(self, state, now):
        self.state = state
        self.history.append({"time": now, "state": state.value,
                             "point_id": self.point_id, "reason": self.reason})

    def _emit(self, kind, payload):
        self.effects.append(Effect(kind, self.token, self.mission_id, self.point_id, payload))

    def _begin(self, state, kind, payload, timeout, now):
        self.token = uuid4().hex
        self.deadline = now + timeout
        self._set(state, now)
        self._emit(kind, payload)

    def start(self, now):
        self._clock(now)
        if self.state != State.IDLE:
            raise RuntimeError("Mission already started; duplicate start rejected")
        self.mission_deadline = now + self.policy.mission_timeout
        self._precheck(now)

    def _precheck(self, now):
        self._begin(State.PRECHECK, "precheck", {"map_id": self.map_id,
                    "capture_home": self.home is None}, self.policy.precheck_timeout, now)

    def resume(self, now):
        self._clock(now)
        if self.state != State.PAUSED:
            raise RuntimeError("Only a safely paused mission can resume")
        if now >= self.mission_deadline:
            self._fail("mission_deadline", now)
        else:
            self.reason = "explicit_resume"
            self._precheck(now)  # Home and already committed points are retained.

    def _navigate(self, now):
        if self.nav_attempts >= 1 + self.policy.nav_retries:
            self._fail("navigation_retry_budget_exhausted", now)
            return
        self.nav_attempts += 1
        returning = self.index == len(self.points)
        pose = self.home if returning else self.points[self.index].pose
        self._begin(State.RETURNING if returning else State.NAVIGATING,
                    "navigate", {"pose": asdict(pose), "map_id": self.map_id,
                                 "attempt": self.nav_attempts}, self.policy.nav_timeout, now)

    def _verify_stop(self, now, next_stage):
        self.retry_stage = next_stage
        self._begin(State.VERIFY_STOP, "hold_and_verify", {
            "pose": asdict(self.home if self.point_id == "HOME" else self.points[self.index].pose),
            "map_id": self.map_id, "next_stage": next_stage}, self.policy.stop_timeout, now)

    def _inspect(self, now):
        if self.inspect_attempts >= 1 + self.policy.inspect_retries:
            self._fail("inspection_retry_budget_exhausted", now)
            return
        self.inspect_attempts += 1
        point = self.points[self.index]
        self._begin(State.INSPECTING, "inspect", {
            "task": asdict(point.task), "pose": asdict(point.pose),
            "map_id": self.map_id, "attempt": self.inspect_attempts},
            self.policy.inspect_timeout, now)

    def _quiesce(self, now, destination, reason):
        old_token, old_state = self.token, self.state.value
        self.reason, self.after_cancel = reason, destination
        self._begin(State.CANCELING, "cancel_and_hold", {
            "old_token": old_token, "old_state": old_state, "reason": reason,
            "require_workers_terminal_and_physical_stop": True},
            self.policy.cancel_timeout, now)

    def _retry(self, stage, reason, now):
        used = self.nav_attempts if stage == "nav" else self.inspect_attempts
        limit = self.policy.nav_retries if stage == "nav" else self.policy.inspect_retries
        self.retry_stage = stage
        self._quiesce(now, "retry" if used <= limit else "failed", reason)

    def _fail(self, reason, now):
        if self.state in self.TERMINAL:
            return
        self.reason, self.token = reason, uuid4().hex
        self.deadline = math.inf
        self._set(State.FAILED, now)
        self._emit("emergency_hold", {"reason": reason, "cancel_all": True})

    def pause(self, now, reason="operator_pause"):
        self._clock(now)
        if self.state in self.TERMINAL or self.state in (State.IDLE, State.PAUSED):
            return
        # An in-flight durable write must be reconciled before any restart.
        if self.state in (State.COMMITTING, State.FINISHING):
            self._fail("storage_outcome_requires_reconciliation:" + reason, now)
        elif self.state == State.CANCELING:
            self.after_cancel, self.reason = "paused", reason
        else:
            self._quiesce(now, "paused", reason)

    def tick(self, now):
        self._clock(now)
        if self.state in self.TERMINAL or self.state == State.IDLE:
            return
        if now >= self.mission_deadline:
            self._fail("mission_deadline", now)
        elif now >= self.deadline:
            if self.state in (State.NAVIGATING, State.RETURNING):
                self._retry("nav", "navigation_timeout", now)
            elif self.state == State.INSPECTING:
                self._retry("inspect", "inspection_timeout", now)
            elif self.state == State.RETRY_WAIT:
                if self.retry_stage == "nav":
                    self._navigate(now)
                else:
                    self._verify_stop(now, "inspect")
            elif self.state == State.PAUSED:
                pass
            else:
                self._fail(self.state.value.lower() + "_timeout", now)

    def handle(self, event, token, now, data=None):
        """Return False for stale, duplicate, out-of-order or unknown events."""
        self.tick(now)  # Deadlines win over late results even if a timer was delayed.
        if token != self.token or self.state in self.TERMINAL:
            return False
        data = {} if data is None else data
        if event == "failed":
            reason = data.get("reason", "adapter_failure")
            if self.state in (State.NAVIGATING, State.RETURNING):
                self._retry("nav", reason, now)
            elif self.state == State.INSPECTING:
                self._retry("inspect", reason, now)
            else:
                self._fail(reason, now)
            return True
        if self.state == State.PRECHECK and event == "precheck_ok":
            if data.get("map_id") != self.map_id or data.get("all_checks_ok") is not True:
                self._fail("precheck_rejected", now)
            else:
                if self.home is None:
                    try:
                        self.home = Pose(**data["home"])
                    except (KeyError, TypeError, ValueError):
                        self._fail("invalid_start_pose", now)
                        return True
                self._navigate(now)
        elif self.state in (State.NAVIGATING, State.RETURNING) and event == "nav_succeeded":
            self._verify_stop(now, "finish" if self.point_id == "HOME" else "inspect")
        elif self.state == State.VERIFY_STOP and event == "stopped":
            if data.get("fresh_stationary_at_target") is not True:
                return False
            if self.retry_stage == "finish":
                self._begin(State.FINISHING, "finish_record", {
                    "home": asdict(self.home), "map_id": self.map_id,
                    "completed_points": [r["point_id"] for r in self.records]},
                    self.policy.commit_timeout, now)
            else:
                self._inspect(now)
        elif self.state == State.INSPECTING and event == "inspection_succeeded":
            spec = self.points[self.index].task
            if not validate_task_result(data, spec, mission_id=self.mission_id,
                                    point_id=self.point_id, token=self.token, map_id=self.map_id):
                self._retry("inspect", "invalid_task_result", now)
            else:
                import copy
                self.pending_record = copy.deepcopy(data)
                self._begin(State.COMMITTING, "commit", {
                    "key": self.mission_id + "/" + self.point_id,
                    "record": self.pending_record}, self.policy.commit_timeout, now)
        elif self.state == State.COMMITTING and event == "committed":
            if data.get("key") != self.mission_id + "/" + self.point_id or data.get("durable") is not True:
                return False
            self.records.append(self.pending_record)
            self.pending_record = None
            self.index += 1  # The ONLY increment: validated task result AND durable commit.
            self.nav_attempts = self.inspect_attempts = 0
            self._navigate(now)
        elif self.state == State.FINISHING and event == "finished":
            if data.get("durable") is not True:
                return False
            self.token, self.deadline = uuid4().hex, math.inf
            self._set(State.COMPLETED, now)
            self._emit("mission_complete", {"home": asdict(self.home), "hold": True})
        elif self.state == State.CANCELING and event == "quiesced":
            if data.get("workers_terminal") is not True or data.get("fresh_stationary") is not True:
                return False  # A cancel request ACK alone does not establish this barrier.
            self.token = uuid4().hex
            if self.after_cancel == "retry":
                self.deadline = now + self.policy.retry_delay
                self._set(State.RETRY_WAIT, now)
            elif self.after_cancel == "paused":
                self.deadline = math.inf
                self._set(State.PAUSED, now)
            else:
                self._fail("retry_budget_exhausted:" + self.reason, now)
        else:
            return False
        return True
