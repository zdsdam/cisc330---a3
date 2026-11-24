"""Ground-truth checks for Tasks 1-4. Run with python -m unittest -v."""

import unittest
from math import sqrt

import numpy as np

from geometry import transform_point
from inverse_kinematics import forward_kinematics, inverse_kinematics
from maximum_targeting_error import MAX_TRE_MM
from mri_to_robot_transform import M_ROBOT, mri_to_robot
from workspace import workspace_limits


TOL = 1e-9
PROSTATE_CENTER = np.array([0.0, -55.0, 55.0])
PROSTATE_RADIUS = 30.0


def check_close(label, actual, expected):
    """Print expected and recovered values, then enforce absolute tolerance."""
    actual = np.asarray(actual)
    expected = np.asarray(expected)
    error = np.max(np.abs(actual - expected))
    print(f"\n  {label}")
    print(f"    expected {np.round(expected, 9).tolist()}")
    print(f"    computed {np.round(actual, 9).tolist()}")
    print(f"    max absolute error {error:.2e}", flush=True)
    np.testing.assert_allclose(actual, expected, rtol=0, atol=TOL)


class RegistrationTests(unittest.TestCase):
    def check_registration(self, markers_mri, expected_transform):
        transform = mri_to_robot(markers_mri)
        mapped = [transform_point(transform, marker) for marker in markers_mri]
        check_close("MRI-to-robot matrix", transform, expected_transform)
        check_close("M1-M3 in robot coordinates (mm)", mapped, M_ROBOT)

    def test_translation(self):
        # MRI pose adds (10, 20, 30) to each robot-frame marker.
        self.check_registration(
            [[10, -40, -70], [10, 20, -70], [10, 10, 10]],
            [[1, 0, 0, -10], [0, 1, 0, -20], [0, 0, 1, -30], [0, 0, 0, 1]],
        )

    def test_rotation(self):
        # The robot-to-MRI rotation maps (x, y, z) to (-y, x, z).
        self.check_registration(
            [[60, 0, -100], [0, 0, -100], [10, 0, -20]],
            [[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        )

    def test_rotation_and_translation(self):
        # Rotate (x, y, z) to (x, -z, y), then add (100, -50, 200).
        self.check_registration(
            [[100, 50, 140], [100, 50, 200], [100, -30, 190]],
            [[1, 0, 0, -100], [0, 0, 1, -200], [0, -1, 0, -50], [0, 0, 0, 1]],
        )

    def test_coincident_frames(self):
        self.check_registration(M_ROBOT, np.eye(4))


class KinematicsTests(unittest.TestCase):
    def check_joints(self, target, expected_joints):
        self.assertLess(np.linalg.norm(np.array(target) - PROSTATE_CENTER),
                        PROSTATE_RADIUS)
        joints = inverse_kinematics(target)
        check_close("inverse (translation mm, rotation deg, depth mm)",
                    joints, expected_joints)
        check_close("forward from hand-computed joints (mm)",
                    forward_kinematics(*expected_joints), target)
        check_close("forward from recovered joints (mm)",
                    forward_kinematics(*joints), target)

    def test_pure_translation(self):
        self.check_joints((0, -50, 60), (10, 0, 50))

    def test_pure_rotation(self):
        self.check_joints((25, -25 * sqrt(3), 50), (0, 30, 50))

    def test_translation_and_rotation(self):
        self.check_joints((-25, -25 * sqrt(3), 60), (10, -30, 50))

    def test_home_and_undefined_axis_rotation(self):
        check_close("home needle tip (mm)", forward_kinematics(0, 0, 0), (0, 0, 0))
        with self.assertRaisesRegex(ValueError, "no unique rotation"):
            inverse_kinematics((0, 0, 0))


class WorkspaceTests(unittest.TestCase):
    def test_joint_extrema(self):
        # Each pair of sphere boundary points attains one joint's two limits.
        half_diagonal = 15 * sqrt(2)
        tangent_x = 30 * sqrt(55**2 - 30**2) / 55
        tangent_y = -(55**2 - 30**2) / 55
        extrema = (
            ((0, -55 - half_diagonal, 55 - half_diagonal),
             (0, -55 + half_diagonal, 55 + half_diagonal)),
            ((-tangent_x, tangent_y, 55), (tangent_x, tangent_y, 55)),
            ((0, -25, 55), (0, -85, 55)),
        )
        keys = ("translation_mm", "rotation_deg", "depth_mm")
        limits = workspace_limits(PROSTATE_CENTER[2])
        for joint, (key, (low, high)) in enumerate(zip(keys, extrema)):
            with self.subTest(joint=key):
                distances = [
                    np.linalg.norm(np.array(point) - PROSTATE_CENTER)
                    for point in (low, high)
                ]
                check_close("witness distances to sphere center (mm)",
                            distances, [PROSTATE_RADIUS, PROSTATE_RADIUS])
                attained = [inverse_kinematics(point)[joint] for point in (low, high)]
                check_close(key, limits[key], attained)

        check_close("physical needle travel (mm)", limits["needle_travel_mm"],
                    [35.35533905932738, 120.20815280171308])
        check_close("translation after a +10 mm axial center shift (mm)",
                    workspace_limits(65)["translation_mm"],
                    np.array(limits["translation_mm"]) + 10)


class TargetingErrorTests(unittest.TestCase):
    def test_maximum_targeting_error(self):
        check_close("maximum targeting error (mm)", MAX_TRE_MM, 3.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
