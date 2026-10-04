#!/usr/bin/env python3
"""Recorded peg-hole demos (peg_hole_demo_recorder.py npz) -> one HIL-SERL demo pkl.

The pkl has the ffw_grasp demo_schema layout the hil-serl learner already loads
({"schema": SCHEMA, "transitions": [...]}, examples/experiments/ffw_grasp/demo_schema.py),
one transition per frame step:

  observations / next_observations  dict:
      "state"        (123,) float32  Frame obs (FRAME.md): what a deployed actor measures
      "priv"         (191,) float32  Frame priv: everything the server derives (critic only)
      "image_right"  (128, 128, 3) uint8 RGB  right wrist D405 ROI (peg)
      "image_left"   (128, 128, 3) uint8 RGB  left wrist D405 ROI (hole)
  actions   (6,) float32 in [-1, 1]: the right-arm delta applied in that step (dx, dy, dz,
            droll, dpitch, dyaw) / (6.67 mm x3, 2 deg x3) -- the server's +-1 per-tick caps,
            so a policy action of 1.0 is one capped step (ControlCmdDelta takes metres/radians:
            multiply back by ACTION_SCALE)
  rewards   float  the server's reward of the step (tag.reward)
  masks     1 - dones;  dones  True on the TERMINATED frame
  infos     {"frame_state", "reason", "episode_id", "step", "succeed", "demo": True}

Step k uses frame k-1 as the observation and frame k's tag (action actually applied
before frame k was measured, its reward, its state). next_observations is the same dict
object as the next transition's observations, so pickle stores each frame once.
Only clean SUCCESS episodes are exported unless --all.

  ../ffw_collision_checker/scripts/.venv/bin/python tools/export_demos.py \\
      ../ffw_collision_checker/scripts/recordings/peg_hole_demos/expert -o peg_hole_demos_100.pkl
"""
import argparse
import glob
import json
import os
import pickle
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffw_peg_hole_env import wire  # noqa: E402

MAX_TRANS, MAX_ROT = 0.01 * 2 / 3, np.radians(2.0)          # peg_hole_serl_server: action +-1
ACTION_SCALE = np.array([MAX_TRANS] * 3 + [MAX_ROT] * 3)
SCHEMA = {
    "format": f"ffw_peg_hole-frame{wire.MSG_FRAME}-v1",
    "obs_keys": ["image_left", "image_right", "priv", "state"],
    "state_shape": (wire.OBS_N,),
    "priv_shape": (wire.PRIV_N,),
    "image_shape": (wire.IMG, wire.IMG, 3),
    "action_shape": (6,),
    "action_scale": ACTION_SCALE.tolist(),
    "obs_fields": [(n, k) for n, k, _ in wire.OBS_FIELDS],
    "priv_fields": [(n, k) for n, k, _ in wire.PRIV_FIELDS],
    "layout_doc": "ffw_peg_hole_env/FRAME.md",
}


def episode_transitions(path):
    z = np.load(path)
    meta = json.loads(str(z["meta"]))
    frames = [wire.decode_frame(f.tobytes()) for f in z["frames"]]
    obs = [{"state": o["vector"].astype(np.float32), "priv": p["vector"].astype(np.float32),
            "image_right": z["img_right"][i], "image_left": z["img_left"][i]}
           for i, (o, p, _, _) in enumerate(frames)]
    out = []
    for k in range(1, len(frames)):
        tag = frames[k][2]
        done = tag["frame_state"] == wire.FS_TERMINATED
        a = np.clip(np.asarray(tag["action"]) / ACTION_SCALE, -1.0, 1.0).astype(np.float32)
        out.append({"observations": obs[k - 1], "actions": a, "next_observations": obs[k],
                    "rewards": float(tag["reward"]), "masks": 0.0 if done else 1.0, "dones": bool(done),
                    "infos": {"frame_state": wire.FS_NAMES[tag["frame_state"]],
                              "reason": wire.TR_NAMES[tag["reason"]], "episode_id": int(tag["episode_id"]),
                              "step": int(tag["step"]), "succeed": bool(done and tag["reason"] == wire.TR_SUCCESS),
                              "demo": True}})
    return out, meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("demo_dirs", nargs="+", help="recorder --demo-dir(s); their episodes/*.npz are read")
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--all", action="store_true", help="also episodes that did not end in a clean SUCCESS")
    a = ap.parse_args()
    files = sorted(f for d in a.demo_dirs for f in glob.glob(os.path.join(d, "episodes", "*.npz")))
    transitions, n_ep, skipped = [], 0, 0
    for f in files:
        tr, meta = episode_transitions(f)
        if not a.all and not (meta.get("reason") == "SUCCESS" and meta.get("interventions", 0) == 0):
            skipped += 1
            continue
        transitions += tr
        n_ep += 1
    with open(a.out, "wb") as fh:
        pickle.dump({"schema": SCHEMA, "transitions": transitions}, fh, protocol=pickle.HIGHEST_PROTOCOL)
    print(f"{n_ep} episodes ({skipped} skipped), {len(transitions)} transitions -> {a.out} "
          f"({os.path.getsize(a.out) / 1e9:.2f} GB)")


if __name__ == "__main__":
    main()
