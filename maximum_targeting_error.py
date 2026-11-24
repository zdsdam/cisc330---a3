"""
maximum_targeting_error.py — compute the error limit for full lesion coverage.

Inputs (from the assignment)
  ABLATION_DIAMETER_MM  12 mm, centered at the actual needle tip
  LESION_DIAMETER_MM     6 mm, centered at the intended target

Output
  MAX_TRE_MM  maximum distance between the two centers (mm)

Plan
  Model both volumes as spheres. If their centers are e mm apart, the
  farthest lesion point is e plus the lesion radius from the needle tip.
  The maximum e is the ablation radius minus the lesion radius.

Boundary case
  * At the 3 mm error limit, the lesion touches the ablation boundary from
    the inside and is still fully covered.

PDF references
  * HW-Robot-Nav.pdf, p. 3, gives the two diameters and the ablation model.
  * MathPrimer-Part1.pdf, p. 22, gives the sphere model used for both volumes.
"""

ABLATION_DIAMETER_MM = 12.0
LESION_DIAMETER_MM = 6.0
MAX_TRE_MM = (ABLATION_DIAMETER_MM - LESION_DIAMETER_MM) / 2
