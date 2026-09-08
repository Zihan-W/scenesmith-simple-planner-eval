#!/usr/bin/env python3
"""Render and numerically validate Zerith simulation camera geometry."""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

from src.online_manipulation import (
    NullTask,
    ObservedBodySpec,
    ScenarioSpec,
    ZerithEnvironmentConfig,
    make_env,
    make_zerith_camera_specs,
)


CAMERA_NAMES = (
    "head_camera",
    "left_wrist_camera",
    "right_wrist_camera",
)
TARGETS = {
    "center_target": {"channel": 0, "camera_xyz_m": (0.0, 0.0, 1.0)},
    "right_target": {"channel": 1, "camera_xyz_m": (0.2, 0.0, 1.0)},
    "down_target": {"channel": 2, "camera_xyz_m": (0.0, 0.15, 1.0)},
}
TARGET_HALF_DEPTH_M = 0.01
RESERVED_LABEL_MIN = 32764


def _rotation_matrix(quaternion_wxyz) -> np.ndarray:
    """Return a 3x3 rotation matrix for one wxyz quaternion."""
    quaternion = np.asarray(quaternion_wxyz, dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    w, x, y, z = quaternion
    return np.array(
        (
            (
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ),
            (
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ),
            (
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ),
        )
    )


def _pose_matrix(pose) -> np.ndarray:
    """Return a homogeneous matrix for one public Pose."""
    matrix = np.eye(4)
    matrix[:3, :3] = _rotation_matrix(pose.quaternion_wxyz)
    matrix[:3, 3] = pose.translation_m
    return matrix


def _point_in_camera(camera, point_world_m) -> np.ndarray:
    """Transform one world point into the camera optical frame."""
    rotation = _rotation_matrix(camera.pose.quaternion_wxyz)
    translation = np.asarray(camera.pose.translation_m, dtype=float)
    return rotation.T @ (np.asarray(point_world_m, dtype=float) - translation)


def _project(camera, point_world_m) -> tuple[float, float, float]:
    """Project one world point using the observation's pinhole intrinsics."""
    x, y, z = _point_in_camera(camera, point_world_m)
    if z <= 0.0:
        raise ValueError(f"Point lies behind camera: z={z}")
    intrinsics = camera.intrinsics
    return (
        intrinsics.focal_x_px * x / z + intrinsics.center_x_px,
        intrinsics.focal_y_px * y / z + intrinsics.center_y_px,
        z,
    )


def _label_for_body(camera, qualified_body_name: str) -> int:
    """Return the render label assigned to one qualified body."""
    matches = [
        label
        for label, name in camera.label_names.items()
        if name == qualified_body_name
    ]
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected one label for {qualified_body_name}, got {matches}"
        )
    return matches[0]


def _label_bbox(label_image: np.ndarray, label: int):
    """Return inclusive xyxy bounds and pixel count for one label."""
    rows, columns = np.where(label_image == label)
    if not len(columns):
        return None
    return {
        "xyxy": [
            int(columns.min()),
            int(rows.min()),
            int(columns.max()),
            int(rows.max()),
        ],
        "center_uv": [
            0.5 * (float(columns.min()) + float(columns.max())),
            0.5 * (float(rows.min()) + float(rows.max())),
        ],
        "pixel_count": int(len(columns)),
    }


def _visible_labels(camera) -> list[dict[str, object]]:
    """List non-reserved labels visible in one image."""
    values, counts = np.unique(camera.label, return_counts=True)
    return [
        {
            "label": int(label),
            "body": camera.label_names.get(int(label), "unmapped"),
            "pixel_count": int(count),
        }
        for label, count in zip(values, counts, strict=True)
        if int(label) < RESERVED_LABEL_MIN
    ]


def _validity_metrics(camera) -> dict[str, object]:
    """Return useful content, depth, label, timestamp, and pose metrics."""
    non_background = camera.label < RESERVED_LABEL_MIN
    finite_depth = np.isfinite(camera.depth) & (camera.depth > 0.0)
    return {
        "non_background_pixel_fraction": float(non_background.mean()),
        "finite_depth_pixel_fraction": float(finite_depth.mean()),
        "visible_labels": _visible_labels(camera),
        "timestamp_s": camera.timestamp_s,
        "X_world_camera_optical": _pose_matrix(camera.pose).tolist(),
    }


def _save_depth_visualization(camera, path: Path) -> None:
    """Save a deterministic blue-to-red metric-depth visualization."""
    depth = camera.depth
    valid = np.isfinite(depth) & (depth > 0.0)
    image = np.zeros((*depth.shape, 3), dtype=np.uint8)
    if np.any(valid):
        low, high = np.percentile(depth[valid], (2.0, 98.0))
        if high <= low:
            high = low + 1.0
        normalized = np.clip((depth - low) / (high - low), 0.0, 1.0)
        image[..., 0] = np.where(valid, 255.0 * normalized, 0.0).astype(
            np.uint8
        )
        image[..., 1] = np.where(
            valid,
            255.0 * (1.0 - np.abs(2.0 * normalized - 1.0)),
            0.0,
        ).astype(np.uint8)
        image[..., 2] = np.where(valid, 255.0 * (1.0 - normalized), 0.0).astype(
            np.uint8
        )
    Image.fromarray(image).save(path)


def _save_label_visualization(camera, path: Path) -> None:
    """Save a deterministic color rendering of the int16 label image."""
    labels = camera.label.astype(np.int64)
    valid = labels < RESERVED_LABEL_MIN
    image = np.zeros((*labels.shape, 3), dtype=np.uint8)
    image[..., 0] = np.where(valid, (53 * labels + 71) % 256, 0)
    image[..., 1] = np.where(valid, (97 * labels + 29) % 256, 0)
    image[..., 2] = np.where(valid, (193 * labels + 113) % 256, 0)
    Image.fromarray(image).save(path)


def _save_camera_images(camera, output_directory: Path, stem: str) -> None:
    """Save RGB, raw metric depth, depth color, and label color outputs."""
    output_directory.mkdir(parents=True, exist_ok=True)
    Image.fromarray(camera.rgb).save(output_directory / f"{stem}_rgb.png")
    np.save(output_directory / f"{stem}_depth_m.npy", camera.depth)
    _save_depth_visualization(
        camera,
        output_directory / f"{stem}_depth_color.png",
    )
    _save_label_visualization(
        camera,
        output_directory / f"{stem}_label_color.png",
    )


def _extrinsics(camera_spec) -> dict[str, list[list[float]]]:
    """Return the two declared mount transforms and their composition."""
    return {
        "X_parent_camera_mount": _pose_matrix(
            camera_spec.X_parent_camera_mount
        ).tolist(),
        "X_mount_camera_optical": _pose_matrix(
            camera_spec.X_mount_camera_optical
        ).tolist(),
        "X_parent_camera_optical": _pose_matrix(
            camera_spec.X_parent_camera_optical
        ).tolist(),
    }


def _calibration_config(repository_root: Path, camera_name: str):
    """Build one self-contained three-target camera calibration config."""
    calibration_root = repository_root / "models" / "zerith_camera_calibration"
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=calibration_root / f"{camera_name}.dmd.yaml",
            package_xmls=(calibration_root / "package.xml",),
            observed_bodies=tuple(
                ObservedBodySpec(name, name, "base_link") for name in TARGETS
            ),
        ),
        robot_model_dir=repository_root / "models" / "zerith_drake",
        robot_xyz=(0.0, 0.0, 0.2315),
        robot_yaw_deg=0.0,
        rail_position=0.4,
        q_home_left=(0.0,) * 7,
        episode_duration=1.0,
        task=NullTask(),
        cameras=make_zerith_camera_specs(
            enabled_names=(camera_name,),
            width=320,
            height=240,
        ),
    )


def validate_calibration(repository_root: Path, output_directory: Path) -> dict:
    """Render all three calibration rigs and validate their projections."""
    results = {}
    for camera_name in CAMERA_NAMES:
        env = make_env(_calibration_config(repository_root, camera_name))
        observation, _ = env.reset(seed=0)
        camera = observation.sensors[camera_name]
        camera_spec = next(
            spec
            for spec in env.backend.adapter.spec.cameras
            if spec.name == camera_name
        )
        _save_camera_images(camera, output_directory, camera_name)
        target_results = {}
        for target_name, target_metadata in TARGETS.items():
            target_pose = observation.objects[target_name].pose
            expected_u, expected_v, center_depth = _project(
                camera,
                target_pose.translation_m,
            )
            label = _label_for_body(
                camera,
                f"{target_name}::base_link",
            )
            bbox = _label_bbox(camera.label, label)
            if bbox is None:
                raise RuntimeError(
                    f"{target_name} is not visible in {camera_name}"
                )
            column = int(round(expected_u))
            row = int(round(expected_v))
            actual_depth = float(camera.depth[row, column])
            expected_depth = center_depth - TARGET_HALF_DEPTH_M
            actual_rgb = camera.rgb[row, column].astype(int)
            dominant_channel = int(np.argmax(actual_rgb))
            pixel_error = math.dist(
                (expected_u, expected_v),
                bbox["center_uv"],
            )
            aligned = bool(
                camera.label[row, column] == label
                and dominant_channel == target_metadata["channel"]
                and np.isfinite(actual_depth)
            )
            target_results[target_name] = {
                "expected_camera_xyz_m": list(
                    target_metadata["camera_xyz_m"]
                ),
                "measured_camera_xyz_m": _point_in_camera(
                    camera,
                    target_pose.translation_m,
                ).tolist(),
                "expected_pixel_uv": [expected_u, expected_v],
                "actual_label_bbox": bbox,
                "pixel_error_px": pixel_error,
                "expected_front_depth_m": expected_depth,
                "actual_depth_m": actual_depth,
                "depth_error_m": abs(actual_depth - expected_depth),
                "render_label": label,
                "rgb_at_expected_pixel": actual_rgb.tolist(),
                "rgb_depth_label_aligned": aligned,
                "passed": bool(
                    pixel_error <= 1.0
                    and abs(actual_depth - expected_depth) <= 1e-3
                    and aligned
                ),
            }
        results[camera_name] = {
            "parent_frame": camera_spec.parent_frame,
            "extrinsics": _extrinsics(camera_spec),
            "intrinsics": camera.intrinsics.as_dict(),
            "validity": _validity_metrics(camera),
            "targets": target_results,
            "passed": all(
                target["passed"] for target in target_results.values()
            ),
        }
    payload = {
        "coordinate_convention": {
            "optical_x": "image right",
            "optical_y": "image down",
            "optical_z": "forward",
        },
        "simulation_camera_intrinsics": True,
        "cameras": results,
        "passed": all(result["passed"] for result in results.values()),
    }
    (output_directory / "calibration_metrics.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    return payload


def _picklift_config(
    repository_root: Path,
    dmd_path: Path,
    scene_package_xml: Path,
    additional_package_xml: Path,
    pick_home_json: Path,
    neck_pitch_rad: float,
):
    """Build the validated PickLift scene directly in PREGRASP."""
    calibration = json.loads(pick_home_json.read_text(encoding="utf-8"))
    q_pregrasp = tuple(calibration["q_pregrasp"])
    return ZerithEnvironmentConfig(
        scenario=ScenarioSpec(
            dmd_path=dmd_path,
            package_xmls=(scene_package_xml, additional_package_xml),
            observed_bodies=(
                ObservedBodySpec(
                    "pick_target",
                    "living_room_box_0",
                    "base_link",
                ),
                ObservedBodySpec(
                    "coffee_table",
                    "living_room_coffee_table_0",
                    "base_link",
                ),
            ),
        ),
        robot_model_dir=repository_root / "models" / "zerith_drake",
        robot_xyz=(2.65, 2.95, 0.1815),
        robot_yaw_deg=180.0,
        rail_position=0.4,
        q_home_left=q_pregrasp,
        locked_joint_position_overrides={
            "neck_pitch_joint": neck_pitch_rad,
        },
        episode_duration=1.0,
        task=NullTask(),
        cameras=make_zerith_camera_specs(
            enabled_names=CAMERA_NAMES,
            width=320,
            height=240,
        ),
    )


def validate_picklift_visibility(
    repository_root: Path,
    output_directory: Path,
    dmd_path: Path,
    scene_package_xml: Path,
    additional_package_xml: Path,
    pick_home_json: Path,
    neck_pitch_rad: float,
) -> dict:
    """Render three cameras at PREGRASP and validate task-region visibility."""
    env = make_env(
        _picklift_config(
            repository_root,
            dmd_path,
            scene_package_xml,
            additional_package_xml,
            pick_home_json,
            neck_pitch_rad,
        )
    )
    observation, _ = env.reset(seed=0)
    results = {}
    for camera_name in CAMERA_NAMES:
        camera = observation.sensors[camera_name]
        _save_camera_images(camera, output_directory, camera_name)
        target_label = _label_for_body(
            camera,
            "living_room_box_0::base_link",
        )
        table_label = _label_for_body(
            camera,
            "living_room_coffee_table_0::base_link",
        )
        results[camera_name] = {
            "validity": _validity_metrics(camera),
            "pick_target_bbox": _label_bbox(camera.label, target_label),
            "coffee_table_bbox": _label_bbox(camera.label, table_label),
        }
    payload = {
        "configuration": {
            "robot_base_xyz_m": [2.65, 2.95, 0.1815],
            "robot_base_yaw_deg": 180.0,
            "rail_position_m": 0.4,
            "neck_pitch_rad": neck_pitch_rad,
            "left_arm_state": "PREGRASP",
        },
        "cameras": results,
        "left_wrist_target_visible": (
            results["left_wrist_camera"]["pick_target_bbox"] is not None
        ),
        "head_workspace_visible": (
            results["head_camera"]["coffee_table_bbox"] is not None
        ),
    }
    payload["passed"] = bool(
        payload["left_wrist_target_visible"]
        and payload["head_workspace_visible"]
    )
    (output_directory / "picklift_visibility_metrics.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )
    return payload


def main() -> None:
    """Run self-contained calibration and optional PickLift visibility."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--repository-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--picklift-dmd", type=Path)
    parser.add_argument("--picklift-scene-package-xml", type=Path)
    parser.add_argument("--pick-home-json", type=Path)
    parser.add_argument(
        "--picklift-additional-package-xml",
        type=Path,
    )
    parser.add_argument("--neck-pitch-rad", type=float, default=0.0)
    args = parser.parse_args()
    repository_root = args.repository_root.resolve()
    calibration_output = args.output_dir.resolve() / "calibration"
    calibration = validate_calibration(repository_root, calibration_output)
    supplied_picklift = (
        args.picklift_dmd,
        args.picklift_scene_package_xml,
        args.pick_home_json,
    )
    if any(value is not None for value in supplied_picklift) and not all(
        value is not None for value in supplied_picklift
    ):
        parser.error(
            "PickLift validation requires --picklift-dmd, "
            "--picklift-scene-package-xml, and --pick-home-json together"
        )
    output = {"calibration": calibration}
    if all(value is not None for value in supplied_picklift):
        additional_package_xml = (
            args.picklift_additional_package_xml.resolve()
            if args.picklift_additional_package_xml is not None
            else repository_root / "models" / "zerith_pick_eval" / "package.xml"
        )
        output["picklift"] = validate_picklift_visibility(
            repository_root,
            args.output_dir.resolve() / "picklift_pregrasp",
            args.picklift_dmd.resolve(),
            args.picklift_scene_package_xml.resolve(),
            additional_package_xml,
            args.pick_home_json.resolve(),
            args.neck_pitch_rad,
        )
    print(json.dumps({name: result["passed"] for name, result in output.items()}))
    if not all(result["passed"] for result in output.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
