"""The vehicle-longitudinal frame, and the sign convention that goes with it.

``lidar_tc`` is yawed about -5.35 deg from the vehicle's longitudinal axis while
``/tf_static`` publishes ``lidar_tc -> base_link`` as IDENTITY. That identity edge is an
uncalibrated placeholder, so ``base_link`` is not the vehicle frame and must not be treated
as one: a consumer reading lidar_tc coordinates as vehicle coordinates inherits 9.4 cm of
lateral error per metre of range (0.9 m at 10 m, 9.4 m at 100 m).

This package does not fight that. The repo does not own ``/tf_static`` -- it comes from the
vehicle driver stack or from bag replay -- and publishing a second ``lidar_tc -> base_link``
edge would be a genuine TF conflict. Instead a NEW leaf frame ``ego`` is published as a
child of ``lidar_tc``, rotated by the correction below. ``base_link`` is left alone and left
wrong.

THE CONSTANT IS NOT INVENTED. Three independent methods agree to 0.19 deg on where the
vehicle axis sits within ``lidar_tc``:

    radar boresight vs direction of travel   -5.484 +/- 0.007 deg
    lane geometry (DEFAULT_EGO_YAW_DEG)      -5.350        deg
    radar boresight vs INS body x-axis       -5.293 +/- 0.007 deg

The radar measurement is the independent confirmation: ``/tf_static`` puts
``delphi_esr_radar`` at yaw -5.443 deg in ``lidar_tc``, and fitting the boresight against
static-target range rate on ``selfcal_loc_2026-09-08_11-47-43`` puts that boresight
+0.041 +/- 0.007 deg from the direction of travel -- i.e. the bumper radar was mounted to
the car, not to the LiDAR's error, so the radar frame IS the vehicle axis to within a
twentieth of a degree. The lane-derived constant falls between the two radar values, so it
is kept rather than replaced, and the residual is now bounded at ~0.1 deg (33 cm of lateral
at 100 m, against the 9.4 m the uncorrected frame costs).

Caveat found in the same fit, worth knowing before anyone trusts ``base_link`` for anything
else: the INS body frame is ITSELF about 0.2 deg off the direction of travel. It reports a
constant +0.212 deg of "sideslip" on straight driving (n=14794, IQR 0.087 deg), stable
across low yaw rates and growing only in hard turns where real sideslip appears. On a
straight, true sideslip is ~0, so that constant is the INS body frame versus travel.

SIGN CONVENTION -- this is the part that is easy to get backwards, so it is stated twice and
tested in both directions:

* "The vehicle axis in lidar_tc is -5.35 deg" means vehicle-FORWARD, expressed in lidar_tc
  coordinates, points at -5.35 deg.
* A TF ``lidar_tc -> ego`` carries the pose of ``ego`` IN ``lidar_tc``, so its yaw is
  ``EGO_YAW_IN_LIDAR_DEG`` = **-5.35**.
* Rotating a POINT from lidar_tc into ego applies the inverse, i.e. **+5.35** deg.

Use :func:`ego_tf_yaw_rad` for the broadcaster and :func:`lidar_to_ego` for points; do not
hand-roll either.
"""

from __future__ import annotations

import math

import numpy as np

# Imported, never restated: this is the same measured constant the lane pipeline already
# uses and already unit-tests. A read-only import adds nothing to perception_common.
from perception_common.lane_geometry import DEFAULT_EGO_YAW_DEG

__all__ = [
    "EGO_YAW_IN_LIDAR_DEG",
    "RADAR_BORESIGHT_VS_TRAVEL_DEG",
    "INS_BODY_VS_TRAVEL_DEG",
    "ego_tf_yaw_rad",
    "rotation_2d",
    "lidar_to_ego",
    "ego_to_lidar",
]

#: Yaw of the ``ego`` frame within ``lidar_tc``, in degrees. This is the value a
#: ``lidar_tc -> ego`` StaticTransformBroadcaster publishes.
EGO_YAW_IN_LIDAR_DEG = DEFAULT_EGO_YAW_DEG

#: Fitted boresight offset of ``delphi_esr_radar`` from the direction of travel, degrees.
#: Near zero is the finding: the radar frame is the vehicle axis.
RADAR_BORESIGHT_VS_TRAVEL_DEG = 0.041

#: Constant apparent sideslip the INS reports on straight driving, degrees. This is the INS
#: body frame versus the direction of travel, not real sideslip.
INS_BODY_VS_TRAVEL_DEG = 0.212


def ego_tf_yaw_rad(correction_deg: float | None = None) -> float:
    """Yaw for the published ``lidar_tc -> ego`` transform, in radians.

    Negative, because it is the pose of ``ego`` expressed in ``lidar_tc``. Pass
    ``correction_deg=0.0`` to reproduce today's (defective) behaviour, which is the rollback.
    """
    deg = EGO_YAW_IN_LIDAR_DEG if correction_deg is None else float(correction_deg)
    return math.radians(deg)


def rotation_2d(yaw_rad: float) -> np.ndarray:
    """Standard 2x2 rotation. ``rotation_2d(a) @ v`` rotates ``v`` by ``+a``."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def lidar_to_ego(xy, correction_deg: float | None = None) -> np.ndarray:
    """Rotate points (N,2) from ``lidar_tc`` into ``ego``.

    This is the INVERSE of the published transform, so the rotation applied is ``+5.35``
    deg while the TF carries ``-5.35``. Translation is zero by construction: ``ego`` shares
    ``lidar_tc``'s origin, because where the vehicle frame's origin belongs (road level? rear
    axle?) is a vehicle-platform decision, not a perception one, and guessing it would inject
    an unmeasured offset into every object.
    """
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64))
    return xy @ rotation_2d(-ego_tf_yaw_rad(correction_deg)).T


def ego_to_lidar(xy, correction_deg: float | None = None) -> np.ndarray:
    """Rotate points (N,2) from ``ego`` back into ``lidar_tc``."""
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64))
    return xy @ rotation_2d(ego_tf_yaw_rad(correction_deg)).T
