"""High-level autonomous navigation intents."""

from __future__ import annotations

from typing import Literal, Protocol

from pydantic import BaseModel, Field

from fleet_contracts.geometry import Pose2D


class PathSegment(BaseModel):
    points: tuple[Pose2D, ...] = Field(description="Sparse poly-line in map frame.")


class PlanningUnavailable(BaseModel):
    kind: Literal["unavailable"]
    explanation: str


class PlanningOutcome(BaseModel):
    kind: Literal["available"]
    path: PathSegment


class NavigationSuccess(BaseModel):
    kind: Literal["success"]
    final_pose_error_m: float | None


class NavigationFailure(BaseModel):
    kind: Literal["failure"]
    code: Literal["localization_lost", "obstacle_blocked", "planner_failed", "internal_error"]
    message: str


class NavigationCancellation(BaseModel):
    kind: Literal["cancelled"]


NavigationResult = NavigationSuccess | NavigationFailure | NavigationCancellation


class GoalNavigator(Protocol):
    def navigate_to(self, *, goal_map: Pose2D) -> NavigationResult: ...

    def cancel_navigation(self) -> NavigationCancellation | None: ...


class PathPlanner(Protocol):
    def plan_global(
        self,
        *,
        start_map: Pose2D,
        goal_map: Pose2D,
    ) -> PlanningOutcome | PlanningUnavailable: ...
