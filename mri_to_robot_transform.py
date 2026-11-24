"""
mri_to_robot_transform.py — compute the MRI-to-robot frame transform.

Inputs (from the assignment)
  markers_mri  M1, M2, M3 observed in MRI coordinates (mm), one per row
  M_ROBOT      the same markers in the shaft-centered robot frame (mm)

Output
  F_robot_mri  4x4 homogeneous transform from MRI to robot coordinates

Plan
  Build the same auxiliary marker frame in MRI and robot coordinates.
  Compose F_robot_marker @ inverse(F_mri_marker), then apply the result
  to a homogeneous MRI point to get its robot coordinates.

Edge cases
  * The three markers must be finite, non-collinear, and in M1–M3 order.
  * Coincident MRI and robot frames give the identity transform.
"""

import numpy as np

from geometry import register


M_ROBOT = np.array([
    [0.0, -60.0, -100.0],  # M1
    [0.0,   0.0, -100.0],  # M2
    [0.0, -10.0,  -20.0],  # M3
])


def mri_to_robot(markers_mri):
    """
    Return the 4x4 MRI-to-robot transform.

    markers_mri: (3, 3) array with M1, M2, M3 as rows in MRI coordinates (mm).
    Apply the result to a homogeneous MRI point to get robot coordinates.
    """
    markers_mri = np.asarray(markers_mri, dtype=float)
    if not np.isfinite(markers_mri).all():
        raise ValueError("MRI marker coordinates must be finite")
    return register(markers_mri, M_ROBOT)
