"""Identity distortion preserves paired pixels without constructing dense maps."""
import numpy as np
import pytest

from uni_robot_agent.integrations.piper_x.environment import observation


@pytest.mark.parametrize("model", ["none", "brown_conrady", "inverse_brown_conrady"])
def test_zero_distortion_preserves_rgb_depth_and_intrinsics(monkeypatch, model):
    def unexpected(*args):
        raise AssertionError("Identity geometry must not allocate rectification maps")

    monkeypatch.setattr(observation, "_rectification_maps", unexpected)
    rgb = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    depth = np.array([[0, .5, 1, 2], [3, 4, 5, 6]], dtype=np.float32)
    intrinsics = {"fx": 120., "fy": 130., "cx": 1.5, "cy": .5, "model": model, "d": [0.] * 5}
    actual_rgb, actual_depth, k = observation.rectify(rgb, depth, intrinsics)
    assert actual_rgb is rgb
    assert actual_depth is depth
    np.testing.assert_array_equal(k, [[120, 0, 1.5], [0, 130, .5], [0, 0, 1]])


@pytest.mark.parametrize("model,coeffs", [("unsupported", [0.] * 5), ("none", [.1, 0, 0, 0, 0])])
def test_identity_shortcut_does_not_accept_invalid_distortion(model, coeffs):
    with pytest.raises(ValueError):
        observation.rectify(np.zeros((2, 2, 3), np.uint8), np.ones((2, 2)),
                           {"fx": 1., "fy": 1., "cx": 0., "cy": 0., "model": model, "d": coeffs})
