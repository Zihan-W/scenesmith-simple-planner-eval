"""Public API for the generic online manipulation environment."""

from src.online_manipulation.actions import (
    CartesianDeltaAction,
    CartesianPoseAction,
    CompositeAction,
    GripperAction,
    HoldAction,
    JointDeltaAction,
    JointPositionAction,
    RobotAction,
    RobotCommand,
    BaseVelocityAction,
)
from src.online_manipulation.contact import (
    CarriedBody,
    FREE_MOTION_CONTACT_POLICY,
    PairContactPolicy,
)
from src.online_manipulation.observations import (
    CameraIntrinsics,
    CameraObservation,
    ContactObservation,
    ObjectObservation,
    Observation,
    Pose,
    RobotObservation,
    SpatialVelocity,
)
from src.online_manipulation.planning import (
    ClearanceMetrics,
    CollisionPair,
    ConfigurationCheck,
    DifferentialIkResult,
    EdgeCheck,
    IkResult,
    PlanningQuery,
    build_planning_query,
)
from src.online_manipulation.factory import make_env
from src.online_manipulation.assembly import run as run_configured_episodes
from src.online_manipulation.evaluation import Evaluator, EvaluatedTask, TaskResultEvaluator
from src.online_manipulation.scene_input import PreparedScene, prepare_scene, inspect_dependencies
from src.online_manipulation.runtime import RuntimeConfig
from src.online_manipulation.base import BaseConfig
from src.online_manipulation.navigation import NavigationConfig, NavigationGoal, Navigator, StaticNavigationMap
from src.online_manipulation.navigation_geometry import build_navigation_map
from src.online_manipulation.adapters.zerith_mobile import ZerithMobileRobotAdapter
from src.online_manipulation.adapters.description import DescriptionRobotAdapter
from src.online_manipulation.adapters.zerith_dual import ZerithDualRobotAdapter, make_zerith_dual_spec
from src.online_manipulation.policies import (
    HoldPolicy,
    JointStepPolicy,
    JointStepPolicyConfig,
    PickLiftPolicy,
    PickLiftPolicyConfig,
)
from src.online_manipulation.runner import (
    EpisodeResult,
    run_episode,
    run_episodes,
)
from src.online_manipulation.environment import OnlineManipulationEnv
from src.online_manipulation.dmd_finalizer import write_updated_dmd
from src.online_manipulation.protocols import (
    ContactPolicy,
    EnvironmentConfig,
    OnlineEnvironment,
    Policy,
    StoppablePolicy,
    RobotAdapter,
    Task,
    TaskEvaluation,
)
from src.online_manipulation.specs import (
    CameraSpec,
    GripperSpec,
    JointSpec,
    ObservedBodySpec,
    PlanarPoseRandomizationSpec,
    RobotSpec,
    RendererSpec,
    ScenarioSpec,
    TimingConfig,
    VisualizationConfig,
)
from src.online_manipulation.tasks import (
    NullTask,
    PickLiftTask,
    PickLiftTaskConfig,
)
from src.online_manipulation.adapters.zerith import (
    ZerithEnvironmentConfig,
    ZerithRobotAdapter,
    make_zerith_camera_specs,
    make_zerith_robot_spec,
)

PUBLIC_API_VERSION = "0.3"

__all__ = [
    "PUBLIC_API_VERSION",
    "CartesianDeltaAction",
    "CartesianPoseAction",
    "CameraIntrinsics",
    "CameraObservation",
    "CameraSpec",
    "CarriedBody",
    "ClearanceMetrics",
    "CollisionPair",
    "CompositeAction",
    "ContactObservation",
    "ContactPolicy",
    "ConfigurationCheck",
    "DifferentialIkResult",
    "EdgeCheck",
    "EnvironmentConfig",
    "EpisodeResult",
    "FREE_MOTION_CONTACT_POLICY",
    "GripperAction",
    "GripperSpec",
    "HoldPolicy",
    "HoldAction",
    "IkResult",
    "JointDeltaAction",
    "JointPositionAction",
    "JointSpec",
    "JointStepPolicy",
    "JointStepPolicyConfig",
    "NullTask",
    "ObjectObservation",
    "Observation",
    "ObservedBodySpec",
    "OnlineManipulationEnv",
    "OnlineEnvironment",
    "PairContactPolicy",
    "PlanarPoseRandomizationSpec",
    "PickLiftTask",
    "PickLiftTaskConfig",
    "PickLiftPolicy",
    "PickLiftPolicyConfig",
    "Policy",
    "PlanningQuery",
    "build_planning_query",
    "make_env",
    "make_zerith_camera_specs",
    "make_zerith_robot_spec",
    "write_updated_dmd",
    "run_episode",
    "run_episodes",
    "Pose",
    "RobotAction",
    "RobotAdapter",
    "RobotObservation",
    "RobotSpec",
    "RendererSpec",
    "ScenarioSpec",
    "SpatialVelocity",
    "Task",
    "TaskEvaluation",
    "TimingConfig",
    "VisualizationConfig",
    "ZerithEnvironmentConfig",
    "ZerithRobotAdapter",
]

# Lazy entry preserves a small import graph; importing the API never loads an
# experiment, expert JSON, optional weight library, or external service.
def load_experiment(*args, **kwargs):
    """Resolve a finite-profile experiment; see experiment.load_experiment."""
    from src.online_manipulation.experiment import load_experiment as load
    return load(*args, **kwargs)


__all__ += [
    "Evaluator", "EvaluatedTask", "TaskResultEvaluator", "PreparedScene",
    "prepare_scene", "inspect_dependencies", "load_experiment", "RuntimeConfig",
    "BaseConfig", "BaseVelocityAction", "RobotCommand", "NavigationConfig",
    "NavigationGoal", "Navigator", "StaticNavigationMap", "build_navigation_map",
    "ZerithMobileRobotAdapter", "ZerithDualRobotAdapter", "make_zerith_dual_spec",
    "DescriptionRobotAdapter", "run_configured_episodes",
]
