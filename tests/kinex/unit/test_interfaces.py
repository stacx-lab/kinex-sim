"""kinex-sim's copy of custom_interfaces against Kinex's."""
from pathlib import Path

import pytest


def test_the_interfaces_are_the_ones_kinex_builds():
    """Both projects build custom_interfaces from their own copy; the same definitions make the same types."""
    project = Path(__file__).resolve().parents[3]
    own = project / "ros2/custom_interfaces"
    kinex = project.parent / "kinex/packages/core/assets/ros/custom_interfaces"
    if not kinex.is_dir():
        pytest.skip("needs a Kinex checkout next to this one")
    names = sorted(str(path.relative_to(own)) for kind in ("action", "srv") for path in (own / kind).iterdir())
    assert names == ["action/Json.action", "srv/Json.srv", "srv/Now.srv", "srv/Wait.srv"]
    assert names == sorted(str(path.relative_to(kinex)) for kind in ("action", "srv") for path in (kinex / kind).iterdir())
    for name in names:
        assert (own / name).read_bytes() == (kinex / name).read_bytes()
