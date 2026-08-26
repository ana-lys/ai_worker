"""Round-trip tests for ffw_zmqinterface.protocol (pure stdlib, no zmq needed)."""

import math

import pytest

from ffw_zmqinterface import protocol as proto


def test_robot_state_round_trip():
    st = proto.RobotState()
    for i in range(proto.N_JOINTS):
        st.joint_pos[i] = float(i)
        st.joint_vel[i] = float(i) * 2.0
        st.joint_acc[i] = float(i) * 3.0
    back, ts = proto.decode_robot_state(proto.encode_robot_state(st))
    assert back.joint_pos == st.joint_pos
    assert back.joint_vel == st.joint_vel
    assert back.joint_acc == st.joint_acc
    assert ts > 0.0


def test_control_round_trip():
    cmd = proto.ControlCmd(
        ((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), (-1.0, -2.0, -3.0, -0.1, -0.2, -0.3))
    )
    ts_send = 1234.567
    back, ts = proto.decode_control(proto.encode_control(cmd, ts=ts_send))
    assert back.ee[0] == pytest.approx(cmd.ee[0])
    assert back.ee[1] == pytest.approx(cmd.ee[1])
    assert ts == pytest.approx(ts_send)


def test_ee_state_round_trip():
    st = proto.EEState(
        ((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), (-1.0, -2.0, -3.0, -0.1, -0.2, -0.3))
    )
    back, ts = proto.decode_ee_state(proto.encode_ee_state(st, ts=99.0))
    assert back.ee[0] == pytest.approx(st.ee[0])
    assert back.ee[1] == pytest.approx(st.ee[1])
    assert ts == pytest.approx(99.0)


def test_head_cam_tf_round_trip():
    # Identity default
    tf0 = proto.HeadCamTf()
    back, ts = proto.decode_head_cam_tf(proto.encode_head_cam_tf(tf0, ts=42.0))
    assert back.matrix == pytest.approx(tf0._IDENTITY)
    assert ts == pytest.approx(42.0)

    # Translation + 90 deg yaw about Z: R maps +X_cam to +Y_control.
    m = (0.0, -1.0, 0.0, 1.0,
         1.0, 0.0, 0.0, 2.0,
         0.0, 0.0, 1.0, 3.0,
         0.0, 0.0, 0.0, 1.0)
    tf1 = proto.HeadCamTf(m)
    back, ts = proto.decode_head_cam_tf(proto.encode_head_cam_tf(tf1, ts=7.0))
    assert back.matrix == pytest.approx(m)
    assert ts == pytest.approx(7.0)


def test_wire_sizes():
    assert proto._HEADER.size == 12                      # int32 type + float64 ts
    assert proto._STATE_STRUCT.size == 3 * proto.N_JOINTS * 8
    assert proto._CTRL_STRUCT.size == 12 * 8
    assert proto._TF_STRUCT.size == 16 * 8
    assert proto._FRAME_SIZES[proto.MSG_ROBOT_STATE] == 12 + 480
    assert proto._FRAME_SIZES[proto.MSG_EE_STATE] == 12 + 96
    assert proto._FRAME_SIZES[proto.MSG_CONTROL_CMD] == 12 + 96
    assert proto._FRAME_SIZES[proto.MSG_HEAD_CAM_TF] == 12 + 128


def test_bad_length_rejected():
    with pytest.raises(ValueError):
        proto.decode_control(b"\x00" * 107)   # wrong length for ControlCmd frame
    with pytest.raises(ValueError):
        proto.decode_robot_state(b"\x00" * 491)
    with pytest.raises(ValueError):
        proto.decode_ee_state(b"\x00" * 107)
    with pytest.raises(ValueError):
        proto.decode_head_cam_tf(b"\x00" * 139)   # 12+128 is 140


def test_type_mismatch_rejected():
    # The type id dispatches the frame: a decoder must reject another type.
    cmd = proto.ControlCmd(((0.0,) * 6, (0.0,) * 6))
    st = proto.EEState(((0.0,) * 6, (0.0,) * 6))
    rs = proto.RobotState()
    tf = proto.HeadCamTf()
    with pytest.raises(ValueError):
        proto.decode_control(proto.encode_ee_state(st))
    with pytest.raises(ValueError):
        proto.decode_control(proto.encode_robot_state(rs))
    with pytest.raises(ValueError):
        proto.decode_ee_state(proto.encode_control(cmd))
    with pytest.raises(ValueError):
        proto.decode_robot_state(proto.encode_control(cmd))
    with pytest.raises(ValueError):
        proto.decode_head_cam_tf(proto.encode_control(cmd))
    with pytest.raises(ValueError):
        proto.decode_head_cam_tf(proto.encode_ee_state(st))
    with pytest.raises(ValueError):
        proto.decode_ee_state(proto.encode_head_cam_tf(tf))


def test_timestamp_makes_frame_unique():
    # Same payload + different sender timestamps -> different bytes, so a
    # byte-dedup cannot swallow a re-asserted goal. Same ts -> identical frame
    # (a stuck sender IS deduped).
    cmd = proto.ControlCmd(((0.0, -0.3, 0.0, 0.0, 0.0, 0.0), (0.0,) * 6))
    a = proto.encode_control(cmd, ts=100.0)
    b = proto.encode_control(cmd, ts=100.5)
    assert a != b
    assert proto.encode_control(cmd, ts=100.0) == a


def test_type_ids_distinct():
    assert len({proto.MSG_ROBOT_STATE, proto.MSG_EE_STATE,
                proto.MSG_CONTROL_CMD, proto.MSG_HEAD_CAM_TF}) == 4


def test_rpy_quat_pure_axes():
    half = math.sqrt(0.5)
    # roll +90deg about X
    assert proto.rpy_to_quat(math.pi / 2, 0, 0) == pytest.approx((half, half, 0, 0))
    # pitch +90deg about Y
    assert proto.rpy_to_quat(0, math.pi / 2, 0) == pytest.approx((half, 0, half, 0))
    # yaw +90deg about Z
    assert proto.rpy_to_quat(0, 0, math.pi / 2) == pytest.approx((half, 0, 0, half))


def test_quat_rpy_pure_axes():
    # Just off the gimbal-lock singularities the decomposition is unique and
    # exact (verified to ~1e-12 even 1 deg from pitch=+/-90 deg).
    near = math.pi / 2 - 0.05
    assert proto.quat_to_rpy(*proto.rpy_to_quat(near, 0, 0)) == pytest.approx(
        (near, 0, 0), abs=1e-12
    )
    assert proto.quat_to_rpy(*proto.rpy_to_quat(0, near, 0)) == pytest.approx(
        (0, near, 0), abs=1e-12
    )
    assert proto.quat_to_rpy(*proto.rpy_to_quat(0, 0, near)) == pytest.approx(
        (0, 0, near), abs=1e-12
    )


def test_quat_rpy_gimbal_lock_equivalent():
    # At pitch = +90 deg exactly, roll/yaw are non-unique: (0, +90, 0) and
    # (180, +90, 180) are the same rotation, so don't assert a specific triple.
    # Just check the returned RPY re-encodes to the same quaternion (up to sign).
    half = math.sqrt(0.5)
    q = (half, 0, half, 0)
    back = proto.quat_to_rpy(*q)
    assert back[1] == pytest.approx(math.pi / 2)
    q2 = proto.rpy_to_quat(*back)
    same = all(a == pytest.approx(b) for a, b in zip(q2, q))
    neg = all(a == pytest.approx(-b) for a, b in zip(q2, q))
    assert same or neg


def test_rpy_quat_round_trip():
    for r, p, y in [(0.3, -0.2, 0.5), (-1.2, 0.4, 0.9), (1.5, -0.6, -2.0),
                    (0.0, 0.0, 0.0), (2.5, 0.1, -1.3)]:
        q = proto.rpy_to_quat(r, p, y)
        back = proto.quat_to_rpy(*q)
        assert back == pytest.approx((r, p, y), abs=1e-12)


def test_rpy_quat_identity():
    assert proto.rpy_to_quat(0, 0, 0) == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert proto.quat_to_rpy(1.0, 0.0, 0.0, 0.0) == pytest.approx((0.0, 0.0, 0.0))
