"""Public API for the generic online manipulation environment."""

from simulation.src.core.actions import (
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
from simulation.src.geometry.contact import (
    CarriedBody,
    FREE_MOTION_CONTACT_POLICY,
    PairContactPolicy,
)
from simulation.src.core.observations import (
    CameraIntrinsics,
    CameraObservation,
    ContactObservation,
    ObjectObservation,
    Observation,
    Pose,
    RobotObservation,
    SpatialVelocity,
)
from simulation.src.geometry.planning import (
    ClearanceMetrics,
    CollisionPair,
    ConfigurationCheck,
    DifferentialIkResult,
    EdgeCheck,
    IkResult,
    PlanningQuery,
    build_planning_query,
)
from simulation.src.runtime.factory import make_env
from simulation.src.runtime.assembly import run as run_configured_episodes
from simulation.src.tasks.evaluation import Evaluator, EvaluatedTask, TaskResultEvaluator
from simulation.src.scene.scene_input import PreparedScene, prepare_scene, inspect_dependencies
from simulation.src.runtime.runtime import RuntimeConfig
from simulation.src.core.base import BaseConfig
from simulation.src.control.navigation import NavigationConfig, NavigationGoal, Navigator, StaticNavigationMap
from simulation.src.geometry.navigation_geometry import build_navigation_map
from simulation.src.robots.adapters.zerith_mobile import ZerithMobileRobotAdapter
from simulation.src.robots.adapters.description import DescriptionRobotAdapter
from simulation.src.robots.adapters.zerith_dual import ZerithDualRobotAdapter, make_zerith_dual_spec
from simulation.src.control.policies import (
    HoldPolicy,
    JointStepPolicy,
    JointStepPolicyConfig,
    PickLiftPolicy,
    PickLiftPolicyConfig,
)
from simulation.src.runtime.runner import (
    EpisodeResult,
    run_episode,
    run_episodes,
)
from simulation.src.runtime.environment import OnlineManipulationEnv
from simulation.src.scene.dmd_finalizer import write_updated_dmd
from simulation.src.core.protocols import (
    ContactPolicy,
    EnvironmentConfig,
    OnlineEnvironment,
    Policy,
    StoppablePolicy,
    RobotAdapter,
    Task,
    TaskEvaluation,
)
from simulation.src.core.specs import (
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
from simulation.src.tasks.tasks import (
    NullTask,
    PickLiftTask,
    PickLiftTaskConfig,
)
from simulation.src.robots.adapters.zerith import (
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
    from simulation.src.io.experiment import load_experiment as load
    return load(*args, **kwargs)


__all__ += [
    "Evaluator", "EvaluatedTask", "TaskResultEvaluator", "PreparedScene",
    "prepare_scene", "inspect_dependencies", "load_experiment", "RuntimeConfig",
    "BaseConfig", "BaseVelocityAction", "RobotCommand", "NavigationConfig",
    "NavigationGoal", "Navigator", "StaticNavigationMap", "build_navigation_map",
    "ZerithMobileRobotAdapter", "ZerithDualRobotAdapter", "make_zerith_dual_spec",
    "DescriptionRobotAdapter", "run_configured_episodes",
]

from simulation.src.runtime.execution import (
    JointGoalExecutionResult, execute_validated_joint_goal,
)

def make_minimal_config(repository_root):
    """Build the repository minimal fixture without expert-policy inputs."""
    from simulation.src.recipes.minimal import make_config
    return make_config(repository_root)

def make_mobile_config(mode, **kwargs):
    """Build a mobile fixture with an explicit repository root and mode."""
    from simulation.src.recipes.mobile import make_config
    return make_config(mode, **kwargs)

__all__ += ["JointGoalExecutionResult", "execute_validated_joint_goal",
            "make_minimal_config", "make_mobile_config"]
