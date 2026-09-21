"""Version-scoped access to sampled Drake RGB-D image timestamps."""

from importlib.metadata import version
from typing import Any

import numpy as np


_IMAGE_TIME_EXPORT_BUG_VERSIONS = frozenset({"1.49.0"})


def _installed_drake_version() -> str:
    """Return the installed Drake distribution version."""
    return version("drake")


def sampled_image_time(
    sensor: Any,
    sensor_context: Any,
    root_context: Any,
) -> float:
    """Return the timestamp held with an RgbdSensorDiscrete image frame.

    Drake 1.49.0 has an output-export defect where the public ``image_time``
    port is abstract-valued and evaluates to the held body pose. Its actual
    vector-valued image-time zero-order hold remains present inside the
    diagram. The workaround is deliberately limited to that exact version;
    unexpected port contracts in other versions fail loudly.

    Args:
        sensor: The owning ``RgbdSensorDiscrete``.
        sensor_context: Context for the discrete sensor diagram.
        root_context: Root Diagram context used to locate child contexts.

    Returns:
        Capture time in simulation seconds.

    Raises:
        RuntimeError: If an unknown Drake version violates the public port
            contract or the 1.49.0 held-time source is ambiguous.
        TypeError: If the known workaround source has the wrong value type.
    """
    value = sensor.image_time_output_port().Eval(sensor_context)
    if isinstance(value, np.ndarray) and value.shape == (1,):
        return float(value[0])

    drake_version = _installed_drake_version()
    if drake_version not in _IMAGE_TIME_EXPORT_BUG_VERSIONS:
        raise RuntimeError(
            "RgbdSensorDiscrete.image_time_output_port() returned "
            f"{type(value).__name__} under Drake {drake_version}; no "
            "version-scoped compatibility workaround is registered"
        )

    time_outputs = []
    for subsystem in sensor.GetSystems():
        if subsystem.num_output_ports() != 1:
            continue
        output = subsystem.get_output_port()
        if output.size() == 1:
            time_outputs.append((subsystem, output))
    if len(time_outputs) != 1:
        raise RuntimeError(
            "Drake 1.49.0 image-time compatibility source is ambiguous: "
            f"found {len(time_outputs)} one-element outputs"
        )
    subsystem, output = time_outputs[0]
    context = subsystem.GetMyContextFromRoot(root_context)
    held_value = output.Eval(context)
    if not isinstance(held_value, np.ndarray) or held_value.shape != (1,):
        raise TypeError(
            "Drake 1.49.0 held image time must be a one-element array, "
            f"got {type(held_value).__name__}"
        )
    return float(held_value[0])
