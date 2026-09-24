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

## Mechanical mount and optical frame

`CameraSpec.parent_frame` is now the direct parent link in the table above.
The fixed joint origin is stored as `X_parent_camera_mount`; it is no longer
hidden by attaching a Drake sensor to the camera child link. A separate
`X_mount_camera_optical` is required for every generic `CameraSpec`, and the
runtime composes:

```text
X_parent_camera_optical = X_parent_camera_mount @ X_mount_camera_optical
```

For these three URDF links, the mesh geometry and fixed-joint orientation show
that the camera child-link axes already use the optical convention: the broad
camera body spans local X/Y, lies primarily behind local Z=0, and its small
positive-Z face points along the rendered viewing direction. Therefore the
Zerith Adapter explicitly sets `X_mount_camera_optical = Identity`. This is a
Zerith simulation calibration choice, not a default supplied
by `CameraSpec` and not a claim about factory hardware calibration.

The resulting transforms are:

```text
left_wrist_pitch_link -> left camera optical
[[-0.000003673, -0.422592305,  0.906319890, 0.1193300],
 [-1.000000000,  0.000001552, -0.000003329, 0.0090000],
 [ 0.000000000, -0.906319890, -0.422592305, 0.0603730],
 [ 0.000000000,  0.000000000,  0.000000000, 1.0000000]]

right_wrist_pitch_link -> right camera optical
[[-0.000003673, -0.422592305,  0.906319890, 0.1193300],
 [-1.000000000,  0.000001552, -0.000003329, 0.0090006],
 [ 0.000000000, -0.906319890, -0.422592305, 0.0603730],
 [ 0.000000000,  0.000000000,  0.000000000, 1.0000000]]

neck_pitch_link -> head camera optical
[[-0.000000000, -0.207911786,  0.978147581,  0.067556857],
 [-1.000000000,  0.000000000, -0.000000000,  0.032500000],
 [ 0.000000000, -0.978147581, -0.207911786, -0.036333207],
 [ 0.000000000,  0.000000000,  0.000000000,  1.000000000]]
```

## Simulation camera assumptions

Neither URDF contains `<sensor>` or Gazebo camera elements. It therefore
provides no verifiable hardware resolution, focal lengths, field of view,
principal point, clipping/depth range, distortion model, frame rate, latency,
or camera optical calibration.

`make_zerith_camera_specs()` configures these defaults:

* resolution: 320×240 pixels;
* vertical field of view: 60 degrees;
* centered pinhole principal point: `(159.5, 119.5)` pixels;
* focal length: `fx = fy = 207.84609690826528` pixels;
* depth range: 0.05–10.0 m;
* update period: 0.05 s (20 Hz);
* modalities: RGB, 32-bit floating-point depth, and 16-bit render label.

These are **simulation camera intrinsics**, not hardware specifications.
Drake's optical convention is +X right, +Y down, and +Z forward. Hardware
intrinsics, distortion, latency, and any measured hardware mount correction
remain unknown and must be supplied before claiming real-camera
correspondence.
