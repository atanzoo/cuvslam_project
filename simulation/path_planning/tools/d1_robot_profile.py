"""Robot-profile contract for the staged D1 path-planning simulation.

方案 A deliberately separates the robot identity from the current execution
backend.  D1 Edu is runnable with the checked-in MuJoCo model.  D1 Max is
represented as an explicit, not-yet-runnable proxy until its imported geometry
is calibrated, planner footprint, velocity limits, and gait interface are
verified.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class RobotProfile:
    """Immutable identity and readiness contract for one robot target."""

    name: str
    model_family: str
    model_asset: str | None
    model_status: str
    footprint_status: str
    gait_status: str
    physics_source: str
    source_revision: str
    runnable: bool


D1_EDU_PROFILE = RobotProfile(
    name="d1_edu",
    model_family="D1 Edu",
    model_asset="simulation/path_planning/models/d1_edu/d1_edu.xml",
    model_status="checked_in_mjcf",
    footprint_status="calibrated_from_mujoco_geoms",
    gait_status="kinematic_pd_proxy",
    physics_source="D1 Edu MuJoCo model; not real-robot evidence",
    source_revision="local-checked-in-model",
    runnable=True,
)


D1_MAX_PROXY_PROFILE = RobotProfile(
    name="d1_max_proxy",
    model_family="D1 Max",
    model_asset="external/Agibot_D1_Max/d1_max.xml",
    model_status="official_urdf_audited_mjcf_generated",
    footprint_status="measured_planner_footprint_pending",
    gait_status="training_in_progress",
    physics_source="AgibotTech/Agibot_D1_Max model in ignored external/; not Git evidence",
    source_revision="83bdebb04ae140f751e7c5e9e2c8be44aad62473",
    runnable=False,
)


_PROFILES = {
    D1_EDU_PROFILE.name: D1_EDU_PROFILE,
    D1_MAX_PROXY_PROFILE.name: D1_MAX_PROXY_PROFILE,
    "d1_max": D1_MAX_PROXY_PROFILE,
}


def available_robot_profiles() -> tuple[str, ...]:
    """Return canonical profile names exposed to command-line tools."""
    return (D1_EDU_PROFILE.name, D1_MAX_PROXY_PROFILE.name)


def get_robot_profile(name: str) -> RobotProfile:
    """Resolve a profile name and reject unknown targets early."""
    try:
        return _PROFILES[str(name)]
    except KeyError as exc:
        choices = ", ".join(available_robot_profiles())
        raise ValueError(
            f"unknown robot profile {name!r}; choose one of: {choices}"
        ) from exc


def require_runnable_profile(name: str) -> RobotProfile:
    """Return a runnable profile or fail before a misleading episode starts."""
    profile = get_robot_profile(name)
    if not profile.runnable:
        raise RuntimeError(
            f"robot profile {profile.name!r} is not runnable yet: "
            f"model={profile.model_status}, "
            f"footprint={profile.footprint_status}, "
            f"gait={profile.gait_status}. "
            "Complete D1 Max geometry, footprint, and gait validation before "
            "enabling D1 Max training."
        )
    return profile
