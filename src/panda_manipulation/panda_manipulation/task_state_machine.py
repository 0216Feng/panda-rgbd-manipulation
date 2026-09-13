"""Deterministic manipulation task state machine."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional


class TaskState(str, Enum):
    IDLE = "IDLE"
    DETECT_OBJECT = "DETECT_OBJECT"
    PLAN_TO_PRE_GRASP = "PLAN_TO_PRE_GRASP"
    APPROACH = "APPROACH"
    CLOSE_GRIPPER = "CLOSE_GRIPPER"
    LIFT = "LIFT"
    PLAN_TO_PLACE = "PLAN_TO_PLACE"
    OPEN_GRIPPER = "OPEN_GRIPPER"
    RETURN_HOME = "RETURN_HOME"
    SUCCESS = "SUCCESS"
    FAILED = "FAILED"


TASK_SEQUENCE = [
    TaskState.DETECT_OBJECT,
    TaskState.PLAN_TO_PRE_GRASP,
    TaskState.APPROACH,
    TaskState.CLOSE_GRIPPER,
    TaskState.LIFT,
    TaskState.PLAN_TO_PLACE,
    TaskState.OPEN_GRIPPER,
    TaskState.RETURN_HOME,
]


@dataclass
class StepResult:
    ok: bool
    reason: str = ""


@dataclass
class TaskReport:
    final_state: TaskState
    history: List[str]
    failures: Dict[str, int]
    failure_reason: Optional[str] = None


@dataclass
class ManipulationTaskStateMachine:
    max_retries: int = 2
    state: TaskState = TaskState.IDLE
    history: List[str] = field(default_factory=list)
    failures: Dict[str, int] = field(default_factory=dict)
    failure_reason: Optional[str] = None

    def reset(self) -> None:
        self.state = TaskState.IDLE
        self.history.clear()
        self.failures.clear()
        self.failure_reason = None

    def run(self, handlers: Dict[TaskState, Callable[[], StepResult]]) -> TaskReport:
        self.state = TaskState.DETECT_OBJECT
        self.history.append(self.state.value)

        for step in TASK_SEQUENCE:
            self.state = step
            if not self.history or self.history[-1] != step.value:
                self.history.append(step.value)
            if not self._run_with_retry(step, handlers.get(step, _default_success)):
                self.state = TaskState.FAILED
                self.history.append(self.state.value)
                return self.report()

        self.state = TaskState.SUCCESS
        self.history.append(self.state.value)
        return self.report()

    def report(self) -> TaskReport:
        return TaskReport(
            final_state=self.state,
            history=list(self.history),
            failures=dict(self.failures),
            failure_reason=self.failure_reason,
        )

    def _run_with_retry(self, step: TaskState, handler: Callable[[], StepResult]) -> bool:
        for attempt in range(self.max_retries + 1):
            result = handler()
            if result.ok:
                return True
            self.failures[step.value] = self.failures.get(step.value, 0) + 1
            self.failure_reason = result.reason or f"{step.value} failed"
            self.history.append(f"{step.value}:retry_{attempt + 1}")
        return False


def _default_success() -> StepResult:
    return StepResult(ok=True)
