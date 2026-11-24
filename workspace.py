"""Joint ranges and maximum targeting error for the assignment geometry."""

from math import asin, degrees, isfinite, sqrt


SHAFT_RADIUS_MM = 15.0
RECTAL_WALL_MM = 10.0
PROSTATE_RADIUS_MM = 30.0
PROSTATE_OFFSET_MM = SHAFT_RADIUS_MM + RECTAL_WALL_MM + PROSTATE_RADIUS_MM

ABLATION_DIAMETER_MM = 12.0
LESION_DIAMETER_MM = 6.0
MAX_TRE_MM = (ABLATION_DIAMETER_MM - LESION_DIAMETER_MM) / 2


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
