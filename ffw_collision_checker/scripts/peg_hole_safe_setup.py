#!/usr/bin/env python3
"""Peg-hole safety setup on the real robot: lift to --lift, hard-lock it, and enforce a
global EE limit profile for both arms built from the REAL peg-hole geometry.

  1. lift_joint -> --lift (default -0.30 m) over the solver's qpos rail
     (peg_hole_teach.lower_lift; asks first unless --yes), then hard-lock it in the IK.
  2. One profile frame per arm, `peg_hole_frame_l` / `peg_hole_frame_r`: origin at the
     teach tool's hole tool point at the reset-box centre (TF base_link), oriented as that
     arm's nominal IK-site goal there (left: the centre hole pose; right: the peg aligned
     on the axis at ON_TOP), so the roll/pitch/yaw bounds are small deviations around 0
     (one shared frame puts the right arm at pitch -90, where they are ill-defined).
     Bounds per arm = the
     IK-site goals (the space joy_hand clamps) of --samples reset poses of the teach
     tool: left = every sampled hole pose; right = every peg start and its aligned path
     CLEAR -> ON_TOP -> bottom of the push; plus --margin-* around that span.
  3. Writes [--name]_l and [--name]_r into config/limit_profiles.txt (one frame per
     section, as ik_solver_cli's "Load global limit profile" expects: load both),
     broadcasts both frames (static TF) and publishes `setg <arm> peg_hole_frame_<arm> ...`
     on /teleop/limit_profile; stays up to keep the frame on TF (the profile fails open
     without it). Ctrl-C: clears nothing, the lift stays locked.

NOTE: joy_hand applies this to SpaceMouse goals only. The IK solver does not clamp
/quest/<arm>/ee_target_pose, which is what peg_hole_teach / peg_hole_serl_server send.

  source ROS; .venv/bin/python peg_hole_safe_setup.py            # asks before moving the lift
  source ROS; .venv/bin/python peg_hole_safe_setup.py --yes
"""
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as Rot
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import peg_hole_stroke as phs
import peg_hole_teach as pht

HERE = Path(__file__).resolve().parent
PROFILE_FILE = HERE.parent / "config" / "limit_profiles.txt"
FRAME, PARENT = "peg_hole_frame", "base_link"          # + "_l" / "_r"


def extract_rpy(R):
    """joy_hand / goal_constraints.hpp convention: R = Rx(roll) Ry(pitch) Rz(yaw)."""
    pitch = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    return np.array([np.arctan2(-R[1, 2], R[2, 2]), pitch, np.arctan2(-R[0, 1], R[0, 0])])


def main():
    ap = pht.build_parser(__doc__)
    ap.add_argument("--yes", action="store_true", help="move the lift without asking")
    ap.add_argument("--name", default="peg_hole_safe_real", help="section name in limit_profiles.txt")
    ap.add_argument("--samples", type=int, default=400)
    ap.add_argument("--margin-left", type=float, nargs=2, default=[0.01, 5.0], help="m, deg around the left span")
    ap.add_argument("--margin-right", type=float, nargs=2, default=[0.02, 8.0], help="m, deg around the right span")
    a = ap.parse_args()
    pht.apply_setup(a)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(True)
    prof_pub = io.create_publisher(String, "/teleop/limit_profile", 10)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 15.0 and not (
                io.raw_js is not None and all(io.ee(s) is not None and io.latest_site(s) is not None
                                              for s in ("left", "right"))):
            time.sleep(0.1)
        if io.raw_js is None or any(io.latest_site(s) is None for s in ("left", "right")):
            print("no /joint_states / TF / achieved EE poses in 15 s - is the teleop running?")
            return
        # 1. lift + lock ------------------------------------------------------------------
        lift = dict(zip(io.raw_js[1], io.raw_js[2]))["lift_joint"]
        if abs(lift - a.lift) > 0.003:
            if not a.yes and input(f"Move the lift {lift:+.3f} -> {a.lift:+.3f} m? Both arms move "
                                   f"{abs(a.lift - lift) * 100:.0f} cm with it; check the space is clear. [y/N] "
                                   ).strip().lower() != "y":
                print("lift not moved; quitting")
                return
            if pht.lower_lift(io, a.lift, a.lift_speed) is None:
                print("lift move failed; quitting")
                return
        else:
            print(f"lift already at {lift:+.3f} m")
        io.set_joint_locked("lift_joint", True)
        print(f"lift_joint hard-locked in the IK solve: {io.joint_locked('lift_joint')}")
        # 2. profile from the real reset geometry --------------------------------------------
        maps = {s: np.linalg.inv(io.ee(s)) @ io.latest_site(s) for s in ("left", "right")}   # TF -> IK site
        tch = pht.Teach.__new__(pht.Teach)
        tch.a, tch.rng, tch.L_center = a, np.random.default_rng(0), pht.sample_center(a)
        roll = Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix()
        aligned = lambda hole_ee, s: (hole_ee @ pht.HOLE_TOOL @ phs.T_from([s, 0, 0], roll)  # noqa: E731
                                      @ np.linalg.inv(pht.PEG_TOOL))
        hole_pt = (tch.L_center @ pht.HOLE_TOOL)[:3, 3]
        frames = {"l": phs.T_from(hole_pt, (tch.L_center @ maps["left"])[:3, :3]),
                  "r": phs.T_from(hole_pt, (aligned(tch.L_center, pht.TOP) @ maps["right"])[:3, :3])}
        Ti = {arm: np.linalg.inv(T) for arm, T in frames.items()}
        pts = {"l": [], "r": []}
        for _ in range(a.samples):
            hole_ee, peg_rel = pht.Teach.sample(tch)
            pts["l"].append(Ti["l"] @ hole_ee @ maps["left"])
            for T in (hole_ee @ peg_rel, aligned(hole_ee, pht.TOP + a.clear), aligned(hole_ee, pht.TOP),
                      aligned(hole_ee, pht.TOP - a.push)):
                pts["r"].append(Ti["r"] @ T @ maps["right"])
        sets = {}
        for arm, (mp, md) in (("l", a.margin_left), ("r", a.margin_right)):
            P = np.array([G[:3, 3] for G in pts[arm]])
            A = np.array([extract_rpy(G[:3, :3]) for G in pts[arm]])
            if np.any(np.abs(A[:, 1]) > np.radians(70)):
                raise SystemExit(f"{arm}: EE pitch in {FRAME}_{arm} near +-90 deg -- roll/yaw bounds ill-defined")
            lo = np.r_[P.min(0) - mp, A.min(0) - np.radians(md)]
            hi = np.r_[P.max(0) + mp, A.max(0) + np.radians(md)]
            sets[arm] = " ".join(f"{v:.6f}" for pair in zip(lo, hi) for v in pair)
            print(f"{arm}: x {lo[0]*100:+.1f}..{hi[0]*100:+.1f}  y {lo[1]*100:+.1f}..{hi[1]*100:+.1f}  "
                  f"z {lo[2]*100:+.1f}..{hi[2]*100:+.1f} cm | roll {np.degrees(lo[3]):+.1f}..{np.degrees(hi[3]):+.1f}  "
                  f"pitch {np.degrees(lo[4]):+.1f}..{np.degrees(hi[4]):+.1f}  "
                  f"yaw {np.degrees(lo[5]):+.1f}..{np.degrees(hi[5]):+.1f} deg")
        text = PROFILE_FILE.read_text() if PROFILE_FILE.exists() else ""
        for arm, vals in sets.items():
            name = f"{a.name}_{arm}"
            section = f"[{name}]\ntype global\nframe {FRAME}_{arm}\nset {arm} {vals}\n"
            pat = re.compile(rf"^\[{re.escape(name)}\]\n(?:(?!\[).*\n?)*", re.M)
            text = pat.sub(section, text) if pat.search(text) else text.rstrip("\n") + "\n\n" + section
        PROFILE_FILE.write_text(text)
        print(f"wrote [{a.name}_l] and [{a.name}_r] to {PROFILE_FILE}")
        # 3. frame on TF + profile to joy_hand ----------------------------------------------
        tfs = []
        for arm, T_f in frames.items():
            t = TransformStamped()
            t.header.stamp = io.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = PARENT, f"{FRAME}_{arm}"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, T_f[:3, 3])
            q = Rot.from_matrix(T_f[:3, :3]).as_quat()
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
            tfs.append(t)
        tf_b = StaticTransformBroadcaster(io)
        tf_b.sendTransform(tfs)
        time.sleep(0.5)
        for _ in range(3):                                   # joy_hand may drop the first on a fresh subscription
            for arm, vals in sets.items():
                prof_pub.publish(String(data=f"setg {arm} {FRAME}_{arm} {vals}"))
            time.sleep(0.2)
        print(f"{FRAME}_l / _r on TF at {np.round(hole_pt, 4)} (parent {PARENT}); profile published to both arms. "
              f"Ctrl-C to quit (the frames then leave TF and joy_hand fails open).")
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nstopped (lift stays locked)")
    finally:
        ex.shutdown()
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
