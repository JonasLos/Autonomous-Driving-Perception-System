"""Guards the vendored copy of the Delphi ESR message definitions.

src/custom_msgs/delphi_esr_driver is an interfaces-only mirror of the real driver package
(https://github.com/JonasLos/delphi_esr_driver), vendored so the perception containers can
deserialize /delphi_esr_interface/radar/tracks without pulling in can_msgs, kvaser_interface
and raptor_dbw_msgs.

It shares the real package's NAME deliberately: a ROS 2 message's DDS type name is
"<package>/msg/<Message>", so a copy under any other name would not match the publisher.
That makes silent drift the failure mode to guard against -- if upstream edits a field and
this copy does not follow, the type hash diverges and the subscription stops matching the
live driver with no error anywhere, just an empty topic.

The upstream checkout is not present in CI, so the comparison is skipped when it is
missing. The structural assertions below always run.
"""

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
VENDORED = REPO / "src" / "custom_msgs" / "delphi_esr_driver" / "msg"
UPSTREAM = Path.home() / "ros_drivers" / "src" / "delphi_esr_driver" / "msg"

MSGS = ("EsrTrack.msg", "EsrTrackArray.msg", "EsrStatus.msg")


@pytest.mark.parametrize("name", MSGS)
def test_vendored_copy_exists(name):
    assert (VENDORED / name).is_file(), f"{name} missing from the vendored package"


@pytest.mark.parametrize("name", MSGS)
def test_matches_upstream_byte_for_byte(name):
    up = UPSTREAM / name
    if not up.is_file():
        pytest.skip(f"upstream checkout not present at {UPSTREAM}")
    assert (VENDORED / name).read_bytes() == up.read_bytes(), (
        f"{name} has drifted from upstream. The DDS type hash is derived from the "
        f"definition, so this WILL silently break the subscription. Re-copy from "
        f"{up} rather than editing the vendored file."
    )


def _fields(path):
    """Field (type, name) pairs, ignoring comments, blanks and constants."""
    out = []
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or "=" in line:
            continue
        parts = re.split(r"\s+", line)
        if len(parts) >= 2:
            out.append((parts[0], parts[1]))
    return out


def test_track_field_order_is_stable():
    """Field ORDER is part of the CDR wire layout, not just the hash.

    Pinned explicitly so a reordering upstream fails here with a readable diff instead of
    surfacing as garbled values at runtime.
    """
    assert _fields(VENDORED / "EsrTrack.msg") == [
        ("std_msgs/Header", "header"),
        ("uint8", "track_id"),
        ("float32", "range"),
        ("float32", "range_rate"),
        ("float32", "angle"),
        ("float32", "amplitude"),
        ("uint8", "track_status"),
        ("bool", "is_cipv"),
        ("bool", "is_merged"),
        ("bool", "is_oncoming"),
        ("bool", "is_new_target"),
        ("bool", "range_rate_ambiguous"),
        ("float32", "lat_rate"),
        ("uint8", "update_count"),
    ]


def test_track_array_is_header_plus_track_sequence():
    assert _fields(VENDORED / "EsrTrackArray.msg") == [
        ("std_msgs/Header", "header"),
        ("EsrTrack[]", "tracks"),
    ]


def test_fields_the_node_reads_are_present():
    """Every EsrTrack field radar_fusion_node touches, asserted in one place."""
    names = {n for _, n in _fields(VENDORED / "EsrTrack.msg")}
    for required in (
        "range", "angle", "range_rate", "amplitude",
        "track_status", "update_count", "track_id",
    ):
        assert required in names, f"radar_fusion_node reads .{required}"


# EsrTrack fields the driver declares but never decodes. is_cipv and range_rate_ambiguous
# are hardcoded false; update_count is 0 on every track in every sweep of the 2026-08-25
# bag. Reading any of them as if it carried information silently drops tracks -- a
# min_update_count of 1 discards 100% of them.
STUB_FIELDS = ("is_cipv", "range_rate_ambiguous")


def _attribute_names(path):
    """Attribute names actually accessed in the file, ignoring comments and docstrings.

    AST rather than substring matching, so documenting a stub field in a docstring -- which
    is exactly where it should be explained -- does not read as depending on it.
    """
    import ast

    return {
        n.attr for n in ast.walk(ast.parse(path.read_text()))
        if isinstance(n, ast.Attribute)
    }


@pytest.mark.parametrize(
    "module", ["radar_fusion_node.py", "radar_geometry.py"]
)
def test_stub_fields_are_not_read(module):
    """Reading an always-false field as a gate would silently drop every track."""
    accessed = _attribute_names(REPO / "src" / "radar_ros" / "radar_ros" / module)
    for stub in STUB_FIELDS:
        assert stub not in accessed, (
            f"{module} reads .{stub}, which the driver never decodes"
        )
