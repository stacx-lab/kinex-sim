"""Triangulation: a physical point's 3D position from its pixels in two or more calibrated camera views.

A view's ray comes from its camera's calibrated geometry and from where the camera was when its image was taken,
so a wrist camera serves from any arm pose. No depth is used.
"""
from itertools import combinations

import numpy as np

from kinex import lib, seconds, world

# The least angle between two views' rays, and the farthest the point may reproject from a view's pixel.
MIN_RAY_ANGLE_DEG, MAX_REPROJECTION_PX = 1., 3.


async def triangulate(views, frame="world:base"):
    """The point that two or more views of it show, in frame.

    A view is (camera, [u, v]) or (camera, [u, v], at): the pixel of the same physical point in that camera's
    image, and at the image message, or its world time in seconds, the pixel was read from, for where the camera
    was then (a wrist camera moves with its arm); without at, the latest. Rays need parallax: the head camera and a
    wrist camera, or one wrist camera from two arm poses. The result is {valid, reasons, point, ray_angle_deg,
    reprojection_errors_px}, with point, [x, y, z] in frame, only when valid: rays at least 1 deg apart, the point
    in front of every camera and within 3 px of every pixel. Low errors do not prove the pixels show one point.
    Reading geometry and frames never advances world time.

    Example: found = await lib.perception.triangulate([("head_camera", [229, 305]), ("left_camera", [185, 323])],
                                                      "left_arm:base")
    """
    if len(views) < 2:
        raise ValueError("triangulation needs two or more views")
    seen = []
    for camera, pixel, *at in views:
        at = at[0] if at else None
        optical = getattr(getattr(at, "header", None), "frame_id", "") or f"{camera}:base"
        if at is not None and not seconds(at):  # tf reads time 0 as the latest: the start, once the world has run
            at = 1e-9 if (await world.now()).time_s else None
        geometry = await lib.images.camera_geometry(camera)
        ray = (await geometry.unproject([pixel]))[0]
        if not np.isfinite(ray).all():
            raise ValueError(f"{camera} has no ray at pixel {list(pixel)}")
        pose = await lib.frames.lookup_transform(frame, optical, at)
        ray = pose.rotation.apply(ray)
        seen.append((geometry, pose, np.asarray(pixel, dtype=float), ray / np.linalg.norm(ray)))
    angle = max(np.degrees(np.arccos(min(1., abs(a[3] @ b[3])))) for a, b in combinations(seen, 2))
    result = {"valid": False, "reasons": [], "ray_angle_deg": float(angle)}
    if angle < MIN_RAY_ANGLE_DEG:
        return {**result, "reasons": ["insufficient_parallax"]}
    # The point nearest every ray in the least-squares sense.
    projectors = [(np.eye(3) - np.outer(ray, ray), pose.translation) for _, pose, _, ray in seen]
    point = np.linalg.solve(sum(p for p, _ in projectors), sum(p @ origin for p, origin in projectors))
    errors, behind = [], False
    for geometry, pose, pixel, _ in seen:
        local = pose.inv().apply(point)
        behind = behind or bool(local[2] <= 0)
        error = np.linalg.norm((await geometry.project([local]))[0] - pixel) if local[2] > 0 else np.nan
        errors.append(float(error) if np.isfinite(error) else None)
    result["reasons"] = (["not_in_front_of_every_camera"] * behind
                         + ["inconsistent_reprojection"] * any(e is None or e > MAX_REPROJECTION_PX for e in errors))
    result.update(valid=not result["reasons"], reprojection_errors_px=errors)
    return {**result, "point": point.tolist()} if result["valid"] else result
