# Zerith Camera Inventory

This inventory was derived from both the upstream URDF at
`models/Zerith_Model/ZERITH_H1_PRO_URDF/urdf/ZR_H1PRO-1.2.00.H.V4.3_URDF_2025.12.02.urdf`
and the generated Drake URDF at
`models/zerith_drake/urdf/zerith_drake.urdf`. The three camera links, parent
joints, and joint origins are identical in both files.

## URDF-provided mechanical mounts

| Public name | URDF camera link | Direct parent link | Parent-to-link xyz (m) | Parent-to-link rpy (rad) | Classification from attachment |
| --- | --- | --- | --- | --- | --- |
| `left_wrist_camera` | `left_jaw_camera_link` | `left_wrist_pitch_link` | `(0.11933, 0.009, 0.060373)` | `(-2.0071, 0, -1.5708)` | Wrist-mounted: fixed directly to the terminal left-wrist link |
| `right_wrist_camera` | `right_jaw_camera_link` | `right_wrist_pitch_link` | `(0.11933, 0.0090006, 0.060373)` | `(-2.0071, 0, -1.5708)` | Wrist-mounted: fixed directly to the terminal right-wrist link |
| `head_camera` | `neck_camera_link` | `neck_pitch_link` | `(0.0675568573382885, 0.0324999999999979, -0.0363332072227294)` | `(-1.78023593389281, 0, -1.5707963267949)` | Head/neck-mounted: fixed directly to the neck-pitch link |

The classification above follows the actual URDF parent attachment; it is not
inferred from a camera name.

The joint chains from `dipan_link` are:

* `left_wrist_camera`: `daogui_joint → body_pitch_joint → body_yaw_joint → left_shoulder_pitch_joint → left_shoulder_roll_joint → left_shoulder_yaw_joint → left_elbow_joint → left_wrist_roll_joint → left_wrist_yaw_joint → left_wrist_pitch_joint → left_jaw_camera_joint (fixed)`.
* `right_wrist_camera`: `daogui_joint → body_pitch_joint → body_yaw_joint → right_shoulder_pitch_joint → right_shoulder_roll_joint → right_shoulder_yaw_joint → right_elbow_joint → right_wrist_roll_joint → right_wrist_yaw_joint → right_wrist_pitch_joint → right_jaw_camera_joint (fixed)`.
* `head_camera`: `daogui_joint → body_pitch_joint → body_yaw_joint → neck_yaw_joint → neck_pitch_joint → neck_camera_joint (fixed)`.

Consequently, each wrist camera follows its corresponding arm, while the head
camera follows neck yaw and pitch. All three also follow the rail and body
joints. The current fixed-base left-arm environment locks the rail at 0.4 m
and locks the body, neck, and right-arm joints; that runtime choice does not
change the URDF kinematic chain.

## Simulation camera assumptions

Neither URDF contains `<sensor>` or Gazebo camera elements. It therefore
provides no verifiable hardware resolution, focal lengths, field of view,
principal point, clipping/depth range, distortion model, frame rate, latency,
or camera optical calibration.

`make_zerith_camera_specs()` uses each URDF camera link as the simulation
optical frame (`X_camera_link_camera = identity`) and configures these defaults:

* resolution: 320×240 pixels;
* vertical field of view: 60 degrees;
* centered pinhole principal point: `(159.5, 119.5)` pixels;
* focal length: `fx = fy = 207.84609690826528` pixels;
* depth range: 0.05–10.0 m;
* update period: 0.05 s (20 Hz);
* modalities: RGB, 32-bit floating-point depth, and 16-bit render label.

These are **simulation camera intrinsics**, not hardware specifications.
Drake's optical convention is +X right, +Y down, and +Z forward. The identity
link-to-optical assumption produces a useful forward/downward view in the
tested Zerith model, but it must be replaced with measured calibration before
claiming hardware correspondence.
