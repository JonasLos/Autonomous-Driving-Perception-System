"""Path setup so the tests import the production modules, not a copy.

Mirrors how test_radar_geometry.py already reaches radar_ros: a sys.path insert rather than a
built overlay, so the suite runs with plain pytest and no colcon build.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
for p in (os.path.join(REPO, "src", "object_fusion"),
          os.path.join(REPO, "src", "perception_common"),
          os.path.join(REPO, "src", "radar_ros")):
    if p not in sys.path:
        sys.path.insert(0, p)
