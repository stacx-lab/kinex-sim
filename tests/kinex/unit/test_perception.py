"""RoboDojo's shipped kinex.lib.perception: triangulation from camera geometry and frame lookups, both stood in."""
import asyncio
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tests.kinex.geometry import available

pytestmark = pytest.mark.skipif(not available(), reason="needs Kinex Python; set KINEX_PYTHON")
MODULE = Path(__file__).resolve().parents[3] / "catalog/robots/robodojo/robodojo-arx-x5/lib/perception.py"
POINT = np.array([.1, -.2, .8])


def looking(origin, target):
    """A camera at origin whose optical +z points at target, +x level."""
    from scipy.spatial.transform import RigidTransform, Rotation
    z = (target - origin) / np.linalg.norm(target - origin)
    x = np.cross(z, [0., 0., 1.])
    x /= np.linalg.norm(x)
    return RigidTransform.from_components(origin, Rotation.from_matrix(np.c_[x, np.cross(z, x), z]))


class Pinhole:
    async def unproject(self, pixels):
        u, v = np.asarray(pixels, float).T
        inside = (0 <= u) & (u < 640) & (0 <= v) & (v < 480)
        return np.where(inside[:, None], np.c_[(u - 320) / 300, (v - 240) / 300, np.ones_like(u)], np.nan)

    async def project(self, points):
        x, y, z = np.asarray(points, float).T
        return np.c_[300 * x / z + 320, 300 * y / z + 240]


@pytest.fixture
def perception(monkeypatch):
    import kinex.lib.frames
    import kinex.lib.images
    cameras = {"head_camera/base": looking(np.array([0., -.41, 1.31]), POINT),
               "left_camera/base": looking(np.array([-.3, -.25, .97]), POINT)}
    lookups = []

    async def lookup_transform(target, source, stamp=None, timeout_s=5):
        lookups.append((target, source, stamp))
        return cameras[source.replace(":", "/")]

    async def camera_geometry(camera):
        return Pinhole()
    monkeypatch.setattr(kinex.lib.frames, "lookup_transform", lookup_transform)
    monkeypatch.setattr(kinex.lib.images, "camera_geometry", camera_geometry)
    spec = importlib.util.spec_from_file_location("perception", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    clock = SimpleNamespace(time_s=0.)

    async def now():
        return clock
    monkeypatch.setattr(module, "world", SimpleNamespace(now=now))
    return module, cameras, lookups, clock


def pixel(camera):
    local = camera.inv().apply(POINT)
    return list(300 * local[:2] / local[2] + [320, 240])


def test_two_views_triangulate_the_point_in_the_frame_asked(perception):
    module, cameras, lookups, _ = perception
    image = SimpleNamespace(header=SimpleNamespace(frame_id="left_camera/base", stamp=SimpleNamespace(sec=3, nanosec=0)))
    found = asyncio.run(module.triangulate([("head_camera", pixel(cameras["head_camera/base"])),
                                            ("left_camera", pixel(cameras["left_camera/base"]), image)], "world:base"))
    assert found["valid"] and found["reasons"] == [] and found["point"] == pytest.approx(POINT, abs=1e-9)
    assert found["reprojection_errors_px"] == pytest.approx([0., 0.], abs=1e-6) and found["ray_angle_deg"] > 1
    # The camera's pose is looked up where its image says, when it was taken.
    assert lookups == [("world:base", "head_camera:base", None), ("world:base", "left_camera/base", image)]


def test_views_without_parallax_or_of_different_points_give_no_point(perception):
    module, cameras, _, _ = perception
    head = pixel(cameras["head_camera/base"])
    same = asyncio.run(module.triangulate([("head_camera", head), ("head_camera", head, 3.2)]))
    assert same == {"valid": False, "reasons": ["insufficient_parallax"], "ray_angle_deg": 0.}
    off = asyncio.run(module.triangulate([("head_camera", head),
                                          ("left_camera", list(np.add(pixel(cameras["left_camera/base"]), [25, 0])))]))
    assert not off["valid"] and off["reasons"] == ["inconsistent_reprojection"] and "point" not in off
    assert max(off["reprojection_errors_px"]) > module.MAX_REPROJECTION_PX
    with pytest.raises(ValueError, match="needs two or more views"):
        asyncio.run(module.triangulate([("head_camera", head)]))
    with pytest.raises(ValueError, match="head_camera has no ray at pixel"):
        asyncio.run(module.triangulate([("head_camera", [700, 10]), ("left_camera", head)]))


def test_a_view_from_world_time_zero_is_the_start_once_the_world_has_run(perception):
    """tf reads time 0 as the latest; the start is still exact until the world runs, then 1 ns after it."""
    module, cameras, lookups, clock = perception
    views = [("head_camera", pixel(cameras["head_camera/base"]), 0.), ("left_camera", pixel(cameras["left_camera/base"]))]
    asyncio.run(module.triangulate(views))
    clock.time_s = 1.2
    asyncio.run(module.triangulate(views))
    assert [stamp for _, source, stamp in lookups if source == "head_camera:base"] == [None, 1e-9]
