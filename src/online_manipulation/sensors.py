"""Shared sampled RGBD systems, independent of robot names and task state."""

import numpy as np
from pydrake.all import (
    BodyIndex,
    CameraInfo,
    ClippingRange,
    DepthRange,
    DepthRenderCamera,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    RgbdSensor,
    RgbdSensorDiscrete,
    RigidTransform,
    Role,
)

from src.online_manipulation._drake_camera_time import sampled_image_time
from src.online_manipulation.adapters.description import drake_pose, public_pose
from src.online_manipulation.observations import CameraObservation


class CameraSystems:
    """Create enabled cameras; sample image, pose and time from one latch."""

    def __init__(self, builder, plant, scene_graph, instance, specs, renderer):
        self.systems = {}
        self.labels = {}
        enabled = tuple(spec for spec in specs if spec.enabled)
        if not enabled:
            return
        scene_graph.AddRenderer(
            renderer.name, MakeRenderEngineVtk(RenderEngineVtkParams())
        )
        for spec in enabled:
            frame = plant.GetFrameByName(spec.parent_frame, instance)
            core = RenderCameraCore(
                renderer.name,
                CameraInfo(spec.width, spec.height, spec.fov_y_rad),
                ClippingRange(spec.near_m, spec.far_m),
                RigidTransform(),
            )
            sensor = builder.AddSystem(
                RgbdSensorDiscrete(
                    RgbdSensor(
                        plant.GetBodyFrameIdOrThrow(frame.body().index()),
                        frame.GetFixedPoseInBodyFrame()
                        @ drake_pose(spec.X_parent_camera_optical),
                        DepthRenderCamera(core, DepthRange(spec.near_m, spec.far_m)),
                        False,
                    ),
                    period=spec.update_period_s,
                    render_label_image="label" in spec.modalities,
                )
            )
            sensor.set_name(spec.name)
            builder.Connect(
                scene_graph.get_query_output_port(), sensor.query_object_input_port()
            )
            self.systems[spec.name] = (spec, sensor)
        inspector = scene_graph.model_inspector()
        for index in range(plant.num_bodies()):
            body = plant.get_body(BodyIndex(index))
            frame_id = plant.GetBodyFrameIdOrThrow(body.index())
            name = f"{plant.GetModelInstanceName(body.model_instance())}::{body.name()}"
            for geometry in inspector.GetGeometries(frame_id, Role.kPerception):
                label = int(
                    inspector.GetPerceptionProperties(geometry).GetProperty(
                        "label", "id"
                    )
                )
                if self.labels.setdefault(label, name) != name:
                    raise RuntimeError(f"Render label {label} has multiple body names")

    def observe(self, root_context):
        """Read held frames without inferring timestamps from current time."""
        observations = {}
        for name, (spec, sensor) in self.systems.items():
            context = sensor.GetMyContextFromRoot(root_context)
            observations[name] = CameraObservation(
                frame=name,
                timestamp_s=sampled_image_time(sensor, context, root_context),
                pose=public_pose(sensor.body_pose_in_world_output_port().Eval(context)),
                intrinsics=spec.intrinsics,
                rgb=(
                    np.asarray(
                        sensor.color_image_output_port().Eval(context).data[:, :, :3],
                        dtype=np.uint8,
                    )
                    if "rgb" in spec.modalities
                    else None
                ),
                depth=(
                    np.asarray(
                        sensor.depth_image_32F_output_port()
                        .Eval(context)
                        .data[:, :, 0],
                        dtype=np.float32,
                    )
                    if "depth" in spec.modalities
                    else None
                ),
                label=(
                    np.asarray(
                        sensor.label_image_output_port().Eval(context).data[:, :, 0],
                        dtype=np.int16,
                    )
                    if "label" in spec.modalities
                    else None
                ),
                label_names=self.labels if "label" in spec.modalities else {},
            )
        return observations
