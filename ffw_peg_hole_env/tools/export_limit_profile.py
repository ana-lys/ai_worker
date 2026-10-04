#!/usr/bin/env python3
"""Export the env's safe range (PegHoleConfig.safety_*) as an ik_solver_cli
GLOBAL limit profile, so teleop is clamped to the same hole-aligned box.

joy_hand clamps its EE goal (the IK site, in the IK/MuJoCo frame it calls
base_link) inside a TF frame, on absolute roll/pitch/yaw of the goal in that
frame. So this tool:
  1. takes the hole frame the env's box uses (zup of the nominal hole site:
     z = insertion axis up, x/y across it) as TF frame `peg_hole_frame`;
  2. derives EE-site bounds in that frame from many env resets + full
     insertions (both arms), plus the env's margins (right: 3 cm across, 2 cm
     along; left: 2 cm; rotation: right +10 deg, left +5 deg around the span);
  3. writes section [peg_hole_safe] (type global, frame peg_hole_frame) into
     ffw_collision_checker/config/limit_profiles.txt -> CLI "Load global limit
     profile" publishes it as `setg l/r peg_hole_frame <12 bounds>`.

  .../.venv/bin/python tools/export_limit_profile.py            # write the profile
  .../.venv/bin/python tools/export_limit_profile.py --publish-tf   # + keep broadcasting peg_hole_frame
The profile only clamps while peg_hole_frame is on TF (joy_hand fails open).
"""
import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import MujocoBackend, PegHoleConfig, PegHoleTask  # noqa: E402
from ffw_peg_hole_env.geometry import pos, rot, zup  # noqa: E402
from ffw_peg_hole_env.peg_hole import rel_aligned  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

PROFILE_FILE = Path(__file__).resolve().parents[2] / "ffw_collision_checker" / "config" / "limit_profiles.txt"
FRAME = "peg_hole_frame"
PARENT = "base_link"            # joy_hand's ee_goal_frame


def extract_rpy(R):
    """joy_hand / goal_constraints.hpp convention: R = Rx(roll) Ry(pitch) Rz(yaw)."""
    pitch = math.asin(max(-1.0, min(1.0, R[0, 2])))
    yaw = math.atan2(-R[0, 1], R[0, 0])
    roll = math.atan2(-R[1, 2], R[2, 2])
    return np.array([roll, pitch, yaw])


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--resets", type=int, default=500)
    ap.add_argument("--name", default="peg_hole_safe")
    ap.add_argument("--publish-tf", action="store_true", help="then keep broadcasting peg_hole_frame (static TF)")
    a = ap.parse_args()

    b = MujocoBackend(cameras=False, timestep=0.004)
    task = PegHoleTask(b, PegHoleConfig())
    c = task.cfg
    T_f = zup(task.hole_nominal)
    Ti = np.linalg.inv(T_f)
    P = {"l": [], "r": []}
    A = {"l": [], "r": []}
    for s in range(a.resets):
        task.reset(seed=s)
        bottom = task.to_ee("right", task.hole_site @ rel_aligned(task.peg_site_height(-0.022), c.roll_deg))
        for arm, T in (("l", task.T_cmd["left"]), ("r", task.T_cmd["right"]), ("r", bottom)):
            G = Ti @ T
            P[arm].append(G[:3, 3])
            A[arm].append(extract_rpy(G[:3, :3]))
    margin = {"r": (0.03, 0.03, 0.02, np.radians(10.0)), "l": (0.02, 0.02, 0.02, np.radians(5.0))}
    lines = ["type global", f"frame {FRAME}"]
    print(f"hole frame (TF {PARENT} -> {FRAME}): pos {np.round(pos(T_f), 4)}, quat xyzw "
          f"{np.round(Rot.from_matrix(rot(T_f)).as_quat(), 6)}")
    for arm in ("l", "r"):
        p, r = np.array(P[arm]), np.array(A[arm])
        mx, my, mz, mr = margin[arm]
        if np.any(np.abs(r[:, 1]) > np.radians(70)):
            raise SystemExit(f"{arm}: EE pitch in the hole frame near +-90 deg -- roll/yaw bounds would be ill-defined")
        lo = np.r_[p.min(0) - [mx, my, mz], r.min(0) - mr]
        hi = np.r_[p.max(0) + [mx, my, mz], r.max(0) + mr]
        vals = [v for pair in zip(lo, hi) for v in pair]
        lines.append(f"set {arm} " + " ".join(f"{v:.6f}" for v in vals))
        print(f"{arm}: x {lo[0]*100:+.1f}..{hi[0]*100:+.1f}  y {lo[1]*100:+.1f}..{hi[1]*100:+.1f}  "
              f"z {lo[2]*100:+.1f}..{hi[2]*100:+.1f} cm | roll {np.degrees(lo[3]):+.1f}..{np.degrees(hi[3]):+.1f}  "
              f"pitch {np.degrees(lo[4]):+.1f}..{np.degrees(hi[4]):+.1f}  yaw {np.degrees(lo[5]):+.1f}..{np.degrees(hi[5]):+.1f} deg")

    # upsert [name] in the CLI's profile file (keep everything else)
    text = PROFILE_FILE.read_text() if PROFILE_FILE.exists() else ""
    section = f"[{a.name}]\n" + "\n".join(lines) + "\n"
    pat = re.compile(rf"^\[{re.escape(a.name)}\]\n(?:(?!\[).*\n?)*", re.M)
    text = pat.sub(section, text) if pat.search(text) else (text.rstrip("\n") + "\n\n" + section)
    PROFILE_FILE.write_text(text)
    print(f"wrote [{a.name}] to {PROFILE_FILE}")

    if a.publish_tf:
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
        rclpy.init()
        node = rclpy.create_node("peg_hole_frame_broadcaster")
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = PARENT, FRAME
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, pos(T_f))
        q = Rot.from_matrix(rot(T_f)).as_quat()
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, q)
        StaticTransformBroadcaster(node).sendTransform(t)
        print(f"broadcasting {PARENT} -> {FRAME} (Ctrl-C to stop; the profile fails open without it)")
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
