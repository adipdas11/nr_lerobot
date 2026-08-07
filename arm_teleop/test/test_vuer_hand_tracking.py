import numpy as np
import pytest

from arm_teleop.vuer_hand_tracking import (
    INDEX_METACARPAL,
    INDEX_TIP,
    PINKY_METACARPAL,
    THUMB_TIP,
    gripper_aperture,
    is_vuer_hand_absent,
    parse_vuer_hand,
    update_hysteresis,
    webxr_to_ros_matrices,
)


def _serialized_identity_hand():
    matrices = np.repeat(np.eye(4)[None, :, :], 25, axis=0)
    return matrices.transpose(0, 2, 1).reshape(-1), matrices


def test_parse_vuer_hand_decodes_each_column_major_matrix():
    values, expected = _serialized_identity_hand()
    expected[0, :3, 3] = [1.0, 2.0, 3.0]
    values = expected.transpose(0, 2, 1).reshape(-1)

    parsed = parse_vuer_hand(values)

    np.testing.assert_allclose(parsed, expected)


def test_parse_vuer_hand_rejects_wrong_length():
    with pytest.raises(ValueError, match="Expected 400"):
        parse_vuer_hand([0.0] * 399)


def test_parse_vuer_hand_decodes_packed_float32_webxr_payload():
    values, expected = _serialized_identity_hand()
    expected[0, :3, 3] = [1.0, 2.0, 3.0]
    packed = expected.astype(np.float32).transpose(0, 2, 1).reshape(-1).tobytes()

    parsed = parse_vuer_hand(packed)

    np.testing.assert_allclose(parsed, expected)


@pytest.mark.parametrize("sentinel", [b"\x00", [b"\x00"]])
def test_vuer_empty_binary_hand_sentinel_is_treated_as_absent(sentinel):
    assert is_vuer_hand_absent(sentinel)


def test_packed_vuer_hand_is_not_treated_as_absent():
    values, _ = _serialized_identity_hand()
    assert not is_vuer_hand_absent(values.astype(np.float32).tobytes())


def test_webxr_to_ros_maps_right_up_backward_to_forward_left_up():
    _, matrices = _serialized_identity_hand()
    matrices[0, :3, 3] = [1.0, 2.0, 3.0]

    converted = webxr_to_ros_matrices(matrices)

    np.testing.assert_allclose(converted[0, :3, 3], [-3.0, -1.0, 2.0])
    np.testing.assert_allclose(converted[0, :3, :3], np.eye(3))


def test_gripper_aperture_is_normalized_by_palm_width():
    _, matrices = _serialized_identity_hand()
    matrices[INDEX_METACARPAL, :3, 3] = [0.0, 0.0, 0.0]
    matrices[PINKY_METACARPAL, :3, 3] = [0.0, 0.08, 0.0]
    matrices[THUMB_TIP, :3, 3] = [0.0, 0.0, 0.0]
    matrices[INDEX_TIP, :3, 3] = [0.0, 0.08, 0.0]

    aperture, ratio = gripper_aperture(matrices, minimum_ratio=0.0, maximum_ratio=2.0)

    assert ratio == pytest.approx(1.0)
    assert aperture == pytest.approx(0.5)


def test_distance_hysteresis_uses_separate_on_and_off_thresholds():
    assert update_hysteresis(False, 0.02, 0.025, 0.035)
    assert update_hysteresis(True, 0.03, 0.025, 0.035)
    assert not update_hysteresis(True, 0.04, 0.025, 0.035)
