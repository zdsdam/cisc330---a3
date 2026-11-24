"""
workspace.py — compute the joint ranges needed to reach the prostate sphere.

Inputs and assumptions
  shaft radius     15 mm
  wall thickness   10 mm
  prostate radius  30 mm, with the sphere tangent to the wall on the -y side
  needle angle     45 degrees from the +z axis
  prostate_z_mm    chosen axial coordinate of the sphere center (mm)

Outputs
  translation_mm    minimum and maximum translation along +z (mm)
  rotation_deg      minimum and maximum rotation about +z (degrees)
  depth_mm          minimum and maximum depth in the handout's model (mm)
  needle_travel_mm  minimum and maximum travel along the needle (mm)

Plan
  Use the sphere's nearest and farthest points for depth, the two rays
  tangent to its xy projection for rotation, and the extrema of z - depth
  for translation. At 45 degrees, needle travel is sqrt(2) times depth.

PDF references
  * HW-Robot-Nav.pdf, pp. 1–2, shows the joint axes and gives the shaft
    diameter, wall thickness, prostate radius, needle angle, and depth model.
  * MathPrimer-Part1.pdf, pp. 7 and 22, gives the cylindrical coordinates
    and sphere equation used in the workspace derivation.
"""

from math import asin, degrees, isfinite, sqrt


SHAFT_RADIUS_MM = 15.0
RECTAL_WALL_MM = 10.0
PROSTATE_RADIUS_MM = 30.0
PROSTATE_OFFSET_MM = SHAFT_RADIUS_MM + RECTAL_WALL_MM + PROSTATE_RADIUS_MM


def workspace_limits(prostate_z_mm):
    """
    Return (min, max) joint ranges for the sphere centered at (0, -55, z_c).

    Depth is the handout's 45-degree depth coordinate. Needle travel is the
    distance along the needle measured from the zero-depth position.
    """
    if not isfinite(prostate_z_mm):
        raise ValueError("prostate center z must be finite")

    depth_min = PROSTATE_OFFSET_MM - PROSTATE_RADIUS_MM
    depth_max = PROSTATE_OFFSET_MM + PROSTATE_RADIUS_MM
    rotation_max = degrees(asin(PROSTATE_RADIUS_MM / PROSTATE_OFFSET_MM))
    translation_center = prostate_z_mm - PROSTATE_OFFSET_MM
    translation_half_range = sqrt(2) * PROSTATE_RADIUS_MM

    return {
        "translation_mm": (
            translation_center - translation_half_range,
            translation_center + translation_half_range,
        ),
        "rotation_deg": (-rotation_max, rotation_max),
        "depth_mm": (depth_min, depth_max),
        "needle_travel_mm": (sqrt(2) * depth_min, sqrt(2) * depth_max),
    }
