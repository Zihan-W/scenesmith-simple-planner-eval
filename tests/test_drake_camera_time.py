"""Tests for version-scoped Drake RGB-D image timestamp access."""

import unittest
from unittest import mock

import numpy as np

from src.online_manipulation import _drake_camera_time


class _OutputPort:
    """Minimal output-port fake."""

    def __init__(self, value, size: int) -> None:
        self._value = value
        self._size = size

    def Eval(self, context):
        del context
        return self._value

    def size(self) -> int:
        return self._size


class _Subsystem:
    """Minimal child-system fake."""

    def __init__(self, output: _OutputPort, output_count: int = 1) -> None:
        self._output = output
        self._output_count = output_count

    def num_output_ports(self) -> int:
        return self._output_count

    def get_output_port(self) -> _OutputPort:
        return self._output

    def GetMyContextFromRoot(self, root_context):
        return root_context


class _Sensor:
    """Minimal RgbdSensorDiscrete fake."""

    def __init__(self, public_value, subsystems=()) -> None:
        self._public_output = _OutputPort(public_value, 1)
        self._subsystems = tuple(subsystems)

    def image_time_output_port(self) -> _OutputPort:
        return self._public_output

    def GetSystems(self):
        return self._subsystems


class DrakeCameraTimeTest(unittest.TestCase):
    """Validate the direct path and the exact Drake 1.49.0 workaround."""

    def test_uses_valid_public_image_time_without_version_workaround(self):
        sensor = _Sensor(np.array([0.25]))
        with mock.patch.object(
            _drake_camera_time,
            "_installed_drake_version",
            side_effect=AssertionError("version lookup should not run"),
        ):
            timestamp = _drake_camera_time.sampled_image_time(
                sensor,
                object(),
                object(),
            )
        self.assertEqual(timestamp, 0.25)

    def test_drake_149_reads_unique_internal_image_time_hold(self):
        held_time = _Subsystem(_OutputPort(np.array([0.15]), 1))
        abstract_output = _Subsystem(_OutputPort(object(), 0))
        sensor = _Sensor(object(), (abstract_output, held_time))
        with mock.patch.object(
            _drake_camera_time,
            "_installed_drake_version",
            return_value="1.49.0",
        ):
            timestamp = _drake_camera_time.sampled_image_time(
                sensor,
                object(),
                object(),
            )
        self.assertEqual(timestamp, 0.15)

    def test_unknown_version_with_invalid_public_port_fails_loudly(self):
        sensor = _Sensor(object())
        with mock.patch.object(
            _drake_camera_time,
            "_installed_drake_version",
            return_value="1.50.0",
        ):
            with self.assertRaisesRegex(
                RuntimeError,
                "Drake 1.50.0.*no version-scoped",
            ):
                _drake_camera_time.sampled_image_time(
                    sensor,
                    object(),
                    object(),
                )

    def test_drake_149_ambiguous_internal_time_source_fails_loudly(self):
        first = _Subsystem(_OutputPort(np.array([0.0]), 1))
        second = _Subsystem(_OutputPort(np.array([0.0]), 1))
        sensor = _Sensor(object(), (first, second))
        with mock.patch.object(
            _drake_camera_time,
            "_installed_drake_version",
            return_value="1.49.0",
        ):
            with self.assertRaisesRegex(RuntimeError, "ambiguous.*found 2"):
                _drake_camera_time.sampled_image_time(
                    sensor,
                    object(),
                    object(),
                )


if __name__ == "__main__":
    unittest.main()
