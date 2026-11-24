"""
inverse_kinematics.py — solve the robot's forward and inverse kinematics.

Inputs (from the assignment)
  translation_mm   signed translation t along the shaft's +z axis (mm)
  rotation_deg     right-handed shaft rotation theta about +z (degrees)
  depth_mm         nonnegative handout depth d (mm), with local tip (0, -d, d)
  target_robot_mm  desired tip position (x, y, z) in the home robot frame (mm)

Outputs
  Forward  needle tip (x, y, z) in the fixed home robot frame (mm)
  Inverse  joint settings (translation_mm, rotation_deg, depth_mm)

Plan
  Forward rotates (0, -d, d) about +z and adds (0, 0, t).
  Inverse uses d = hypot(x, y), theta = atan2(x, -y), and t = z - d.

Edge cases
  * Zero depth puts the tip on the shaft axis for any shaft rotation.
  * An inverse target on that axis has no unique rotation and raises ValueError.
"""

import numpy as np

from geometry import rot_z


def forward_kinematics(translation_mm, rotation_deg, depth_mm):
    """
    Return the needle tip (x, y, z) in the home robot frame, in mm.

    translation_mm is signed along +z. rotation_deg is right-handed about +z.
    depth_mm is the handout's nonnegative depth d, with local tip (0, -d, d).
    """
    if not np.isfinite([translation_mm, rotation_deg, depth_mm]).all():
        raise ValueError("joint settings must be finite")
    if depth_mm < 0:
        raise ValueError("insertion depth must be nonnegative")

    home_tip = np.array([0.0, -depth_mm, depth_mm])
    return rot_z(rotation_deg) @ home_tip + np.array([0.0, 0.0, translation_mm])


def inverse_kinematics(target_robot_mm):
    """
    Return (translation_mm, rotation_deg, depth_mm) for a home-frame target.

    Rotation uses the principal atan2 angle. An on-axis target has no unique
    rotation and raises ValueError.
    """
    target = np.asarray(target_robot_mm, dtype=float)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError("target must contain three finite coordinates")
    x, y, z = target

    depth_mm = np.hypot(x, y)
    if depth_mm == 0:
        raise ValueError("a target on the shaft axis has no unique rotation")

    translation_mm = z - depth_mm
    rotation_deg = np.degrees(np.arctan2(x, -y))
    return float(translation_mm), float(rotation_deg), float(depth_mm)
