"""Round-trip tests for ffw_zmqinterface.protocol (pure stdlib, no zmq needed)."""

import math

import pytest

from ffw_zmqinterface import protocol as proto


def test_obs_round_trip():
    obs = proto.Obs(
        ((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), (-1.0, -2.0, -3.0, -0.1, -0.2, -0.3)),
        gripper=(0.0, 1.0),
    )
    for i in range(proto.N_JOINT_STATE):
        obs.joint_pos[i] = float(i)
        obs.joint_vel[i] = float(i) * 2.0
        obs.joint_effort[i] = float(i) * 0.5
    ts_send = 1234.567
    back, ts = proto.decode_obs(proto.encode_obs(obs, ts=ts_send))
    assert back.ee[0] == pytest.approx(obs.ee[0])
    assert back.ee[1] == pytest.approx(obs.ee[1])
    assert back.gripper == pytest.approx((0.0, 1.0))
    assert back.joint_pos == obs.joint_pos
    assert back.joint_vel == obs.joint_vel
    assert back.joint_effort == obs.joint_effort
    assert ts == pytest.approx(ts_send)


def test_obs_joints_lead_ee_block_at_tail():
    # The Obs payload mirrors a recorded state.csv row: the joint block leads
    # (index 0 == arm_l_joint1), the 14-double EE + grippers block (_ctrl_values
    # flattening) sits at the tail -- NOT a byte-identical prefix anymore.
    obs = proto.Obs(
        ((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), (-1.0, -2.0, -3.0, -0.1, -0.2, -0.3)),
        gripper=(0.0, 1.0),
    )
    for i in range(proto.N_JOINT_STATE):
        obs.joint_pos[i] = float(i)
        obs.joint_vel[i] = float(i) * 2.0
        obs.joint_effort[i] = float(i) * 0.5
    frame = proto.encode_obs(obs, ts=42.0)
    payload = frame[proto._HEADER.size:]
    import struct
    n = proto.N_JOINT_STATE
    tail = 3 * n  # EE block starts after the pos + vel + effort blocks
    # Leading 3n doubles are exactly joint_pos + vel + effort.
    lead = struct.unpack_from("<%dd" % tail, payload, 0)
    assert lead == pytest.approx(obs.joint_pos + obs.joint_vel + obs.joint_effort)
    # Last 14 doubles are the EE + grippers block in _ctrl_values order.
    ee_tail = struct.unpack_from("<14d", payload, tail * 8)
    assert ee_tail == pytest.approx(proto._ctrl_values(obs.ee, obs.gripper))


def test_obs_default_blocks_zero():
    # A fresh Obs is all zeros in the joint block and open grippers -- the
    # same defaults drain_ee consumers already saw in EEState.
    back, _ = proto.decode_obs(proto.encode_obs(proto.Obs(), ts=1.0))
    assert back.ee[0] == pytest.approx((0.0,) * 6)
    assert back.gripper == pytest.approx((0.0, 0.0))
    assert back.joint_pos == [0.0] * proto.N_JOINT_STATE
    assert back.joint_vel == [0.0] * proto.N_JOINT_STATE
    assert back.joint_effort == [0.0] * proto.N_JOINT_STATE


def test_obs_wire_size():
    assert len(proto.encode_obs(proto.Obs(), ts=1.0)) == 724


def test_obs_bad_length_rejected():
    with pytest.raises(ValueError):
        proto.decode_obs(b"\x00" * 723)   # 12 + 712 is 724
    with pytest.raises(ValueError):
        proto.decode_obs(b"\x00" * 725)


def test_priv_round_trip():
    # Identity default matrix, all-zero delta.
    p0 = proto.Priv()
    back, ts = proto.decode_priv(proto.encode_priv(p0, ts=42.0))
    assert back.matrix == pytest.approx(p0._IDENTITY)
    assert back.delta[0] == pytest.approx((0.0,) * 6)
    assert back.delta[1] == pytest.approx((0.0,) * 6)
    assert ts == pytest.approx(42.0)

    # Translation + 90 deg yaw about Z: R maps +X_cam to +Y_control.
    m = (0.0, -1.0, 0.0, 1.0,
         1.0, 0.0, 0.0, 2.0,
         0.0, 0.0, 1.0, 3.0,
         0.0, 0.0, 0.0, 1.0)
    p1 = proto.Priv(
        matrix=m,
        delta=((0.01, -0.02, 0.03, 0.1, 0.0, 0.0), (-0.01, 0.0, 0.0, 0.0, -0.2, 0.0)),
    )
    back, ts = proto.decode_priv(proto.encode_priv(p1, ts=7.0))
    assert back.matrix == pytest.approx(m)
    assert back.delta[0] == pytest.approx(p1.delta[0])
    assert back.delta[1] == pytest.approx(p1.delta[1])
    assert ts == pytest.approx(7.0)


def test_priv_wire_size():
    assert len(proto.encode_priv(proto.Priv(), ts=1.0)) == 236


def test_priv_bad_length_rejected():
    with pytest.raises(ValueError):
        proto.decode_priv(b"\x00" * 235)   # 12 + 224 is 236
    with pytest.raises(ValueError):
        proto.decode_priv(b"\x00" * 237)


def test_control_round_trip():
    cmd = proto.ControlCmd(
        ((1.0, 2.0, 3.0, 0.1, 0.2, 0.3), (-1.0, -2.0, -3.0, -0.1, -0.2, -0.3)),
        gripper=(0.0, 1.0),
    )
    ts_send = 1234.567
    back, ts = proto.decode_control(proto.encode_control(cmd, ts=ts_send))
    assert back.ee[0] == pytest.approx(cmd.ee[0])
    assert back.ee[1] == pytest.approx(cmd.ee[1])
    assert back.gripper == pytest.approx((0.0, 1.0))
    assert ts == pytest.approx(ts_send)


def test_control_default_gripper_open():
    # A pose-only controller omits the gripper -> it defaults to open (0.0),
    # so existing controllers keep working without moving the grippers.
    cmd = proto.ControlCmd(((1.0, 0.0, 0.0, 0.0, 0.0, 0.0), (0.0,) * 6))
    back, _ = proto.decode_control(proto.encode_control(cmd, ts=1.0))
    assert back.gripper == pytest.approx((0.0, 0.0))


def test_wire_sizes():
    assert proto._HEADER.size == 12                      # int32 type + float64 ts
    assert proto._CTRL_STRUCT.size == 14 * 8             # two 6-DOF poses + 2 grippers
    assert proto._OBS_STRUCT.size == (14 + 3 * proto.N_JOINT_STATE) * 8
    assert proto._PRIV_STRUCT.size == (16 + 12) * 8
    assert proto._OVERRIDE_STRUCT.size == proto.N_JOINT_STATE * 8  # 25 doubles
    assert proto._FRAME_SIZES[proto.MSG_OBS] == 12 + 712  # 724 B
    assert proto._FRAME_SIZES[proto.MSG_CONTROL_CMD] == 12 + 112  # 124 B
    assert proto._FRAME_SIZES[proto.MSG_PRIV] == 12 + 224  # 236 B
    assert proto._FRAME_SIZES[proto.MSG_OVERRIDE] == 12 + 200  # 212 B


def test_override_round_trip():
    # The 25-double joint-space rail: round-trips a per-index ramp, preserves
    # the sender ts, and defaults to a full all-zero commanded vector.
    ramp = [float(i) for i in range(proto.N_JOINT_STATE)]
    back, ts = proto.decode_override(proto.encode_override(proto.OverrideCmd(ramp), ts=55.0))
    assert back.joint_pos == pytest.approx(ramp)
    assert ts == pytest.approx(55.0)

    # Default is an all-zero commanded pose (all joints home), length exact.
    home = proto.OverrideCmd()
    back, ts = proto.decode_override(proto.encode_override(home, ts=1.0))
    assert back.joint_pos == [0.0] * proto.N_JOINT_STATE
    assert len(back.joint_pos) == proto.N_JOINT_STATE

    # Constructor copies its input, so mutating the caller's list afterwards
    # cannot change a built frame's content.
    src = [0.0] * proto.N_JOINT_STATE
    cmd = proto.OverrideCmd(src)
    src[0] = 99.0
    assert cmd.joint_pos[0] == 0.0


def test_override_wire_size():
    assert len(proto.encode_override(proto.OverrideCmd(), ts=1.0)) == 212


def test_override_bad_length_rejected():
    with pytest.raises(ValueError):
        proto.decode_override(b"\x00" * 211)  # 12+200 is 212
    with pytest.raises(ValueError):
        proto.decode_override(b"\x00" * 213)
    # Encoder contract: the rail carries exactly N_JOINT_STATE joint positions.
    with pytest.raises(ValueError):
        proto.encode_override(proto.OverrideCmd([0.0] * 24))
    with pytest.raises(ValueError):
        proto.encode_override(proto.OverrideCmd([0.0] * (proto.N_JOINT_STATE + 1)))


def test_override_order_matches_joint_state_names():
    # OverrideCmd's 25 doubles are the commanded joint positions in exactly
    # JOINT_STATE_NAMES / dataset order -- index N lands on the same joint as
    # Obs joint index N. Encode a per-index ramp, decode, and check each
    # position lines up with its named joint.
    vals = [float(i) for i in range(proto.N_JOINT_STATE)]
    back, _ = proto.decode_override(proto.encode_override(proto.OverrideCmd(vals)))
    for i, name in enumerate(proto.JOINT_STATE_NAMES):
        assert back.joint_pos[i] == float(i), (
            f"OverrideCmd index {i} ({name}) out of order")
    # The left-arm chain leads and the base wheels trail -- spot-check the
    # two ends of the vector.
    assert proto.JOINT_STATE_NAMES[0] == "arm_l_joint1"
    assert proto.JOINT_STATE_NAMES[-1] == "right_wheel_steer"


def test_joint_state_names_contract():
    # The fixed joint order is the wire contract: 25 unique names, no dups.
    assert proto.N_JOINT_STATE == 25
    assert len(proto.JOINT_STATE_NAMES) == proto.N_JOINT_STATE
    assert len(set(proto.JOINT_STATE_NAMES)) == proto.N_JOINT_STATE


def test_joint_state_names_match_dataset_order():
    # Wire joint order == the recorded state.csv q_* column order (left-arm
    # chain first, grippers after both arms, head, then wheels + lift), so an
    # Obs joint block lines up 1:1 with a dataset row.
    names = proto.JOINT_STATE_NAMES
    assert names[0] == "arm_l_joint1"
    assert names[6] == "arm_l_joint7"
    assert names[7] == "arm_r_joint1"
    assert names[13] == "arm_r_joint7"
    assert names[14] == "gripper_l_joint1"
    assert names[15] == "gripper_r_joint1"
    assert names[16:18] == ("head_joint1", "head_joint2")
    assert names[18] == "left_wheel_drive"
    assert names[19] == "left_wheel_steer"
    assert names[20] == "lift_joint"
    assert names[21] == "rear_wheel_drive"
    assert names[22] == "rear_wheel_steer"
    assert names[23] == "right_wheel_drive"
    assert names[24] == "right_wheel_steer"


def test_grip_joint_indices():
    # The Obs joint block is addressed by JOINT_STATE_NAMES index; in the
    # dataset order the grippers are at 14 (left) and 15 (right) -- right arm
    # follows the left arm chain.
    assert proto.GRIP_R_JOINT_IDX == 15
    assert proto.GRIP_L_JOINT_IDX == 14
    assert proto.JOINT_STATE_NAMES[proto.GRIP_R_JOINT_IDX] == "gripper_r_joint1"
    assert proto.JOINT_STATE_NAMES[proto.GRIP_L_JOINT_IDX] == "gripper_l_joint1"


def test_obs_joint_block_offset():
    # The 25-joint x3 block leads the payload (starts at double index 0,
    # mirroring a dataset row) and slices cleanly.
    obs = proto.Obs()
    for i in range(proto.N_JOINT_STATE):
        obs.joint_pos[i] = float(i) + 100.0   # distinct from the zeroed vel
    back, _ = proto.decode_obs(proto.encode_obs(obs, ts=1.0))
    assert back.joint_pos == obs.joint_pos
    assert back.joint_vel == [0.0] * proto.N_JOINT_STATE
    assert back.joint_effort == [0.0] * proto.N_JOINT_STATE


def test_bad_length_rejected():
    with pytest.raises(ValueError):
        proto.decode_control(b"\x00" * 123)   # wrong length for ControlCmd frame
    with pytest.raises(ValueError):
        proto.decode_obs(b"\x00" * 723)
    with pytest.raises(ValueError):
        proto.decode_priv(b"\x00" * 235)
    with pytest.raises(ValueError):
        proto.decode_override(b"\x00" * 211)


def test_gripper_clamped():
    assert proto.clamp01(-0.5) == 0.0
    assert proto.clamp01(0.0) == 0.0
    assert proto.clamp01(0.5) == 0.5
    assert proto.clamp01(1.0) == 1.0
    assert proto.clamp01(2.0) == 1.0
    assert proto.clamp01(0.3) == pytest.approx(0.3)


def test_type_mismatch_rejected():
    # The type id dispatches the frame: a decoder must reject another type.
    obs = proto.Obs(((0.0,) * 6, (0.0,) * 6))
    cmd = proto.ControlCmd(((0.0,) * 6, (0.0,) * 6))
    priv = proto.Priv()
    ovr = proto.OverrideCmd([0.0] * proto.N_JOINT_STATE)
    with pytest.raises(ValueError):
        proto.decode_control(proto.encode_obs(obs))
    with pytest.raises(ValueError):
        proto.decode_obs(proto.encode_control(cmd))
    with pytest.raises(ValueError):
        proto.decode_priv(proto.encode_obs(obs))
    with pytest.raises(ValueError):
        proto.decode_obs(proto.encode_priv(priv))
    with pytest.raises(ValueError):
        proto.decode_control(proto.encode_priv(priv))
    with pytest.raises(ValueError):
        proto.decode_priv(proto.encode_control(cmd))
    with pytest.raises(ValueError):
        proto.decode_obs(proto.encode_override(ovr))
    with pytest.raises(ValueError):
        proto.decode_priv(proto.encode_override(ovr))
    with pytest.raises(ValueError):
        proto.decode_override(proto.encode_obs(obs))


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
    assert len({proto.MSG_OBS, proto.MSG_CONTROL_CMD,
                proto.MSG_PRIV, proto.MSG_OVERRIDE, proto.MSG_RECORD}) == 5
    # v2 ids keep the v1 numbers for the surviving rails so a v1 control/
    # override sender still speaks the same wire format.
    assert proto.MSG_CONTROL_CMD == 2
    assert proto.MSG_OVERRIDE == 4


def test_record_round_trip():
    for value in (0, 1, 2, 3):
        rec = proto.Record(value)
        data = proto.encode_record(rec, ts=42.0)
        assert len(data) == proto._FRAME_SIZES[proto.MSG_RECORD]
        decoded, ts = proto.decode_record(data)
        assert decoded.value == value
        assert ts == 42.0


def test_record_wrong_type_rejected():
    obs = proto.Obs()
    with pytest.raises(ValueError):
        proto.decode_record(proto.encode_obs(obs))
    with pytest.raises(ValueError):
        proto.decode_obs(proto.encode_record(proto.Record(1)))


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
