"""Map MRI coordinates to the shaft-centered frame using M1, M2, and M3."""

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
