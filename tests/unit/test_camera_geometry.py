import numpy as np
import pytest

from rosbridge.catalog.cameras import color, distort, undistort

# HandsOn's head camera: the Gemini 336's calibrated lens (k1, k2, p1, p2, k3).
LENS = (0.00646656053141, -0.0479530096054, -0.000352804054273, -0.000104829683551, 0.0332237109542)


def test_undistort_inverts_the_lens_across_the_image():
    x, y = np.meshgrid(np.linspace(-1., 1., 21), np.linspace(-.6, .6, 13))
    np.testing.assert_allclose(distort(*undistort(x, y, LENS), LENS), (x, y), atol=1e-12)
    assert np.abs(np.subtract(undistort(x, y, LENS), (x, y))).max() > 1e-3


def test_color_states_a_lens_and_its_field_of_view_or_an_undistorted_pinhole():
    pinhole = color(854, 480, 463., 463., 426.5, 239.5)
    assert pinhole["distortion"] == {"model": "none"} and "fov" not in pinhole and pinhole["fps"] is None
    lensed = color(854, 480, 463., 463., 426.5, 239.5, LENS)
    assert lensed["distortion"].pop("model") == "opencv_brown_conrady"
    assert lensed["distortion"] == pytest.approx(dict(zip(("k1", "k2", "p1", "p2", "k3"), LENS)))
    # A zero lens has the field of view Kinex derives for the pinhole; this lens widens it.
    flat = color(854, 480, 463., 463., 426.5, 239.5, (0.,) * 5)["fov"]
    assert flat["horizontal_deg"] == pytest.approx(2 * np.degrees(np.arctan(427 / 463.)))
    assert flat["vertical_deg"] == pytest.approx(2 * np.degrees(np.arctan(240 / 463.)))
    assert lensed["fov"]["horizontal_deg"] > flat["horizontal_deg"]
