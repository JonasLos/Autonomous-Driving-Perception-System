"""A projection PointCloud2 that may carry a per-point ``ground`` flag, parsed on first use.

The same role as ``perception_common.stamp_sync.ProjectedCloud``, which is used unchanged
elsewhere, extended with one optional field. It is a separate class rather than an edit to
perception_common because that package is shared and not this stack's to modify.

A cloud without the ``ground`` field -- transform.py's /lidar_2d_projection -- parses with
``ground = None``, and the camera detector then behaves exactly as it did before segmentation
existed. So pointing the detector back at the old topic is a complete rollback.
"""

from __future__ import annotations

import numpy as np

__all__ = ["GroundFlaggedCloud", "decode_fields"]


def decode_fields(msg, names):
    """Read float32 fields by name straight from a PointCloud2's buffer.

    Used instead of sensor_msgs_py because it also works on the dynamically-generated message
    classes mcap_ros2 yields offline, which sensor_msgs_py rejects. Returns ``None`` for a field
    the cloud does not carry.
    """
    offs = {f.name: f.offset for f in msg.fields}
    step = int(msg.point_step)
    n = (len(msg.data) // step) if step else 0
    if n == 0:
        return {k: (np.empty(0, np.float32) if k in offs else None) for k in names}
    a = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: n * step].reshape(n, step)
    return {k: (a[:, offs[k]:offs[k] + 4].copy().view(np.float32).ravel() if k in offs else None)
            for k in names}


class GroundFlaggedCloud:
    __slots__ = ("msg", "_parsed")

    def __init__(self, msg):
        self.msg = msg
        self._parsed = None

    @property
    def header(self):
        return self.msg.header

    def arrays(self):
        """``(xyz Nx3, u, v, ground-or-None)``, index-aligned, NaN rows removed."""
        if self._parsed is None:
            f = decode_fields(self.msg, ("x", "y", "z", "u", "v", "ground"))
            if f["x"] is None or f["u"] is None:
                e = np.empty(0, np.float32)
                self._parsed = (np.empty((0, 3), np.float32), e, e, None)
            else:
                xyz = np.stack([f["x"], f["y"], f["z"]], axis=1)
                ok = np.isfinite(xyz).all(axis=1) & np.isfinite(f["u"]) & np.isfinite(f["v"])
                g = None if f["ground"] is None else (f["ground"][ok] > 0.5)
                self._parsed = (xyz[ok], f["u"][ok], f["v"][ok], g)
        return self._parsed
