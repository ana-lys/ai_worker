#!/usr/bin/env python3
"""Record expert peg-hole demos on the real robot for HIL-SERL (REAL_ROBOT_README.md plan C).

The demos go through the SERL server's own episode code (peg_hole_serl_server.SerlServer):
same reset, same clamps, same machine interventions, same Frame (obs + priv + tag), with
the expert in the policy's place. Every episode frame (POLICY / INTERVENTION / TERMINATED)
is saved with both 128x128 wrist images (images.WristCameras, the ROI from
peg_hole_roi_gui.py).

Start configurations: a fixed scrambled Sobol sequence (--sobol-seed) over the reset space,
11 dims -- hole across the axis (y, z +- --extent), height along it (--height), tilt about
both cross axes (+- --tilt-range); peg start 2-4 cm above the rim, +- --xy across, +- --tilt
about 3 axes -- taken in order; reach-check failures are skipped. Evenly covers every axis
at any count (a 6+ dim grid of 100 points can't).

Expert: the peg-tool offset from the fitted model (config/peg_hole_offset_model.json,
--offset model | const), over to the hole axis --hover above the rim at --approach-speed,
settle, then straight down the axis at --insert-speed. A demo counts only if it ends
SUCCESS with no machine intervention (--allow-interventions to keep those too).

Any other ending -> manual calibration at that pose: the peg goes up the axis, then to
--hover above the rim at the model's offset, orientation held aligned to the live axis;
you correct it with the right SpaceMouse (translation) and press Enter in the teach window.
That alignment goes into the offset pool (config/peg_hole_offset_pool.json; one entry per
demo pose, a re-calibration replaces it), the model is refit and saved, and the same pose
is retaken. Keys in the teach window: Enter = record alignment, s = skip this pose,
q = quit after this step, space = stop now.

Resumable: <demo-dir>/index.json keeps the plan and every pose's status; a rerun continues.
Output: <demo-dir>/episodes/pose_KKK.npz (the counted demos), failed/pose_KKK_tryN.npz
(the rest), each: frames (N, wire.FRAME_BYTES) uint8 raw Frames (wire.decode_frame),
img_right / img_left (N, 128, 128, 3) RGB uint8, img_ok (N, 2), meta (JSON string).

  source ROS; .venv/bin/python peg_hole_safe_setup.py        # once (keep it running)
  source ROS; .venv/bin/python peg_hole_demo_recorder.py --allow-motion
"""
import json
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import rclpy.signals
from rclpy.executors import SingleThreadedExecutor
from scipy.spatial.transform import Rotation as Rot
from scipy.stats import qmc

import peg_hole_random as phr
import peg_hole_serl_server as srvmod
import peg_hole_stroke as phs
import peg_hole_teach as pht
from peg_hole_serl_server import HZ, MAX_ROT, MAX_TRANS, GuardTrip, HoleMoved, SerlServer
from ffw_peg_hole_env import images, wire
from ffw_peg_hole_env.geometry import clip_delta, delta_between, pose_error

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config"
POOL_FILE = CONFIG / "peg_hole_offset_pool.json"
MODEL_FILE = srvmod.OFFSET_MODEL
ALIGN_INIT = HERE / "recordings" / "peg_hole_align" / "20261004_152716" / "align.json"
DIMS = ("hole_y", "hole_z", "hole_height", "hole_tilt_y", "hole_tilt_z",
        "peg_s", "peg_y", "peg_z", "peg_tilt_x", "peg_tilt_y", "peg_tilt_z")
KEEP = (wire.FS_POLICY, wire.FS_INTERVENTION, wire.FS_TERMINATED)


class Quit(Exception):
    pass


# --- offset pool + model ------------------------------------------------------------------------
def features(e):
    """Model features of a pool entry / pose: 1, grid y, z, hole height, hole tilt y, z."""
    return np.array([1.0, *e["grid_m"], e["hole_height_m"], *e["hole_tilt_deg"]])


def load_pool():
    if POOL_FILE.exists():
        return json.loads(POOL_FILE.read_text())
    d = json.loads(ALIGN_INIT.read_text())
    entries = [{"id": f"align:{ALIGN_INIT.parent.name}:{r['pose']}", "grid_m": r["grid_m"],
                "hole_height_m": r["hole_height_m"], "hole_tilt_deg": r["hole_tilt_deg"],
                "y_mm": r["commanded_mm"]["y"], "z_mm": r["commanded_mm"]["z"], "time": d["time"]}
               for r in d["results"]]
    pool = {"source": str(ALIGN_INIT.relative_to(HERE.parent)), "entries": entries}
    POOL_FILE.write_text(json.dumps(pool, indent=1) + "\n")
    print(f"offset pool started from {ALIGN_INIT} ({len(entries)} eye alignments) -> {POOL_FILE}")
    return pool


def fit(pool):
    """The align GUI's model: peg-tool y / z offset [mm] linear in features(); const = mean."""
    E = pool["entries"]
    X = np.array([features(e) for e in E])
    Y = np.array([[e["y_mm"], e["z_mm"]] for e in E])
    cy = np.linalg.lstsq(X, Y[:, 0], rcond=None)[0]
    cz = np.linalg.lstsq(X, Y[:, 1], rcond=None)[0]
    return {"source": f"{POOL_FILE.name} ({len(E)} alignments, refit "
                      f"{datetime.now().isoformat(timespec='seconds')})",
            "frame": "peg tool y/z offset [mm]",
            "features": ["1", "grid_y_m", "grid_z_m", "height_m", "tilt_y_deg", "tilt_z_deg"],
            "y": cy.tolist(), "z": cz.tolist(), "const": Y.mean(0).tolist()}


def predict(model, pose, kind):
    if kind == "const":
        return np.array(model["const"], float)
    f = features(pose)
    return np.array([f @ model["y"], f @ model["z"]])


# --- poses ---------------------------------------------------------------------------------------
def plan_poses(a, n=512):
    """Scrambled Sobol points in [0, 1)^11, fixed by --sobol-seed (first n, taken in order)."""
    return qmc.Sobol(d=len(DIMS), scramble=True, seed=a.sobol_seed).random_base2(int(np.ceil(np.log2(n))))


def pose_from(u, a, t):
    """Sobol point -> pose dict (model features + the reset pair)."""
    lerp = lambda lo, hi, x: lo + (hi - lo) * x  # noqa: E731
    gy, gz = lerp(-a.extent, a.extent, u[0]), lerp(-a.extent, a.extent, u[1])
    hx = lerp(*a.height, u[2])
    tilt = [lerp(-a.tilt_range, a.tilt_range, u[3]), lerp(-a.tilt_range, a.tilt_range, u[4])]
    H0 = t.L_center @ pht.HOLE_TOOL
    H = H0.copy()
    H[:3, 3] = H0[:3, 3] + H0[:3, :3] @ np.array([hx, gy, gz])
    H[:3, :3] = H0[:3, :3] @ Rot.from_euler("yz", tilt, degrees=True).as_matrix()   # the align GUI's tilt
    hole_ee = H @ np.linalg.inv(pht.HOLE_TOOL)
    s0 = pht.TOP + lerp(0.02, 0.04, u[5])                                             # Teach.sample_peg's ranges
    py, pz = lerp(-a.xy, a.xy, u[6]), lerp(-a.xy, a.xy, u[7])
    ptilt = [lerp(-a.tilt, a.tilt, x) for x in u[8:11]]
    rel = phs.T_from([s0, py, pz], Rot.from_euler("xyz", ptilt, degrees=True).as_matrix()
                     @ Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix())
    peg_rel = pht.HOLE_TOOL @ rel @ np.linalg.inv(pht.PEG_TOOL)
    return {"u": [float(x) for x in u], "grid_m": [gy, gz], "hole_height_m": hx, "hole_tilt_deg": tilt,
            "peg_start": {"s_above_top_m": s0 - pht.TOP, "y_m": py, "z_m": pz, "tilt_deg": ptilt},
            "pair": (hole_ee, peg_rel)}


# --- expert --------------------------------------------------------------------------------------
class Expert:
    """Over the (offset) hole axis at hover, settle, then straight down it."""

    def __init__(self, srv, a, off_mm):
        self.srv, self.a = srv, a
        self.off = phs.T_from([0.0, off_mm[0] / 1000.0, off_mm[1] / 1000.0], np.eye(3))
        self.phase, self.settled, self.n_int = "approach", 0, 0
        self.s = pht.TOP + a.hover

    def target(self, s):
        t = self.srv.t
        return t.peg_target(s, self.off)[0] @ t.st.arms["right"].map

    def delta(self):
        srv, a = self.srv, self.a
        if srv.em.interventions != self.n_int:               # the machine stepped in: start over from hover
            self.n_int, self.phase, self.settled, self.s = srv.em.interventions, "approach", 0, pht.TOP + a.hover
        if self.phase == "approach":
            tgt = self.target(pht.TOP + a.hover)
            e = pose_error(srv.T_cmd, tgt)
            self.settled = self.settled + 1 if e[0] < 2e-4 and e[1] < np.radians(0.2) else 0
            if self.settled >= a.settle_ticks:
                self.phase = "insert"
            return clip_delta(delta_between(srv.T_cmd, tgt), a.approach_speed / HZ, MAX_ROT)
        self.s = max(self.s - a.insert_speed / HZ, pht.TOP - srv.t.a.push - 0.002)
        return clip_delta(delta_between(srv.T_cmd, self.target(self.s)), MAX_TRANS, MAX_ROT)


# --- server with frame capture -------------------------------------------------------------------
class DemoServer(SerlServer):
    def __init__(self, *args, **kw):
        super().__init__(*args, **kw)
        self.buf = None

    def publish(self):
        ok = super().publish()
        if ok and self.buf is not None and self.state in KEEP:
            imgs = self.last_images or {}
            blank = np.zeros((images.OUT, images.OUT, 3), np.uint8)
            self.buf.append((np.frombuffer(self.last_frame, np.uint8).copy(),
                             imgs.get("right") if imgs.get("right") is not None else blank,
                             imgs.get("left") if imgs.get("left") is not None else blank,
                             [imgs.get("right") is not None, imgs.get("left") is not None]))
        return ok


class Recorder:
    def __init__(self, srv, a):
        self.srv, self.t, self.a = srv, srv.t, a
        self.dir = Path(a.demo_dir)
        (self.dir / "episodes").mkdir(parents=True, exist_ok=True)
        (self.dir / "failed").mkdir(exist_ok=True)
        self.index_file = self.dir / "index.json"
        if self.index_file.exists():
            self.index = json.loads(self.index_file.read_text())
            print(f"resuming {self.dir}: {self.n_done()} / {self.index['plan']['n']} demos done")
        else:
            self.index = {"plan": {"n": a.n, "sobol_seed": a.sobol_seed, "dims": DIMS, "extent_m": a.extent,
                                   "height_m": a.height, "tilt_range_deg": a.tilt_range, "xy_m": a.xy,
                                   "tilt_deg": a.tilt, "hover_m": a.hover, "insert_speed": a.insert_speed,
                                   "approach_speed": a.approach_speed, "offset": a.offset, "lift": a.lift,
                                   "started": datetime.now().isoformat(timespec="seconds")}, "poses": {}}
        p = self.index["plan"]
        for k in ("sobol_seed", "extent_m", "height_m", "tilt_range_deg", "xy_m", "tilt_deg"):
            arg = {"sobol_seed": a.sobol_seed, "extent_m": a.extent, "height_m": a.height,
                   "tilt_range_deg": a.tilt_range, "xy_m": a.xy, "tilt_deg": a.tilt}[k]
            if json.dumps(p[k]) != json.dumps(arg):
                raise SystemExit(f"{self.index_file}: plan {k} = {p[k]}, args say {arg}; use the same args or "
                                 f"another --demo-dir")
        self.U = plan_poses(a, max(512, 4 * p["n"]))
        self.pool = load_pool()
        self.model = json.loads(MODEL_FILE.read_text()) if MODEL_FILE.exists() else fit(self.pool)

    def n_done(self):
        return sum(1 for v in self.index["poses"].values() if v["status"] == "done")

    def save_index(self):
        self.index_file.write_text(json.dumps(self.index, indent=1) + "\n")

    def save_episode(self, path, meta):
        b = self.srv.buf or []
        np.savez_compressed(path, frames=np.array([x[0] for x in b], np.uint8).reshape(-1, wire.FRAME_BYTES),
                            img_right=np.array([x[1] for x in b], np.uint8).reshape(-1, images.OUT, images.OUT, 3),
                            img_left=np.array([x[2] for x in b], np.uint8).reshape(-1, images.OUT, images.OUT, 3),
                            img_ok=np.array([x[3] for x in b], bool).reshape(-1, 2),
                            meta=np.array(json.dumps(meta)))

    def key(self):
        k, self.t.pending_key = self.t.pending_key, None
        return k

    # --- one demo pose -------------------------------------------------------------------------
    def episode(self, k, pose, attempt, off):
        srv = self.srv
        srv.set_offset(off)
        srv.next_pair = pose["pair"]
        srv.episode_id = 1000 * (k + 1) + attempt - 1          # reset() starts episode 1000 (k + 1) + attempt
        srv.running, srv.paused, srv.buf = True, False, []
        if not srv.reset():
            srv.buf = None
            return None
        ex = Expert(srv, self.a, off)
        while True:
            srv.delta, srv.delta_t = ex.delta(), time.monotonic()
            if not srv.episode_tick():
                break
        info = dict(srv.info)
        res = {"episode_id": srv.episode_id, "reason": wire.TR_NAMES[srv.reason], "steps": srv.step,
               "interventions": int(srv.em.interventions), "return": float(info.get("return", np.nan)),
               "push_back_peak": float(info.get("push_back_peak", np.nan)), "offset_mm": off.tolist(),
               "offset_kind": self.a.offset, "attempt": attempt, "frames": len(srv.buf),
               "time": datetime.now().isoformat(timespec="seconds")}
        return res

    def manual(self, k, pose, off):
        """Manual calibration at this pose -> 'enter' (recorded + refit) / 's' / 'q'."""
        srv, t, a = self.srv, self.t, self.a
        srv.state = wire.FS_IDLE
        while True:
            try:
                srv.retract()
                OFF = phs.T_from([0.0, off[0] / 1000.0, off[1] / 1000.0], np.eye(3))
                T, _ = t.peg_target(pht.TOP + a.hover, OFF)
                t.move_right(T, a.speed, "align_hover")
                rel_tool = phs.T_from([pht.TOP, 0.0, 0.0], Rot.from_euler("x", pht.ROLL, degrees=True).as_matrix())
                R_al = (self.io_left() @ pht.HOLE_TOOL @ rel_tool)[:3, :3]
                t.manual_trans = R_al @ np.array([a.hover, off[0] / 1000.0, off[1] / 1000.0])
                t.state = "manual"
                print(f"  MANUAL: align the peg by eye with the right SpaceMouse (model said y {off[0]:+.2f} "
                      f"z {off[1]:+.2f} mm), Enter in the teach window = record; s = skip pose; q = quit")
                self.key()
                srv.watch = srv.hole_now()
                try:
                    while True:
                        t.manual_tick()
                        kk = self.key()
                        if kk in ("\r", "\n"):
                            break
                        if kk in ("s", "q"):
                            return kk
                finally:
                    srv.watch, t.state = None, "idle"
                o = t.taught_offset()
                y, z = o[1, 3] * 1000.0, o[2, 3] * 1000.0
                self.add_to_pool(k, pose, y, z)
                print(f"  recorded y {y:+.2f} z {z:+.2f} mm (model said {off[0]:+.2f} {off[1]:+.2f}); "
                      f"pool {len(self.pool['entries'])}, model refit: const {self.model['const'][0]:+.2f} "
                      f"{self.model['const'][1]:+.2f} mm")
                return "enter"
            except (HoleMoved, GuardTrip, phr.LagTrip) as e:
                print(f"  HOLD: {e} -- r: redo the manual alignment, s: skip pose, q: quit")
                self.key()
                while True:
                    t.st.tick(None)
                    kk = self.key()
                    if kk in ("s", "q"):
                        return kk
                    if kk == "r":
                        break

    def io_left(self):
        return self.srv.io.ee("left")

    def add_to_pool(self, k, pose, y, z):
        eid = f"demo:{self.dir.name}:{k}"
        e = {"id": eid, "grid_m": pose["grid_m"], "hole_height_m": pose["hole_height_m"],
             "hole_tilt_deg": pose["hole_tilt_deg"], "y_mm": y, "z_mm": z,
             "time": datetime.now().isoformat(timespec="seconds")}
        self.pool["entries"] = [x for x in self.pool["entries"] if x["id"] != eid] + [e]
        POOL_FILE.write_text(json.dumps(self.pool, indent=1) + "\n")
        self.model = fit(self.pool)
        MODEL_FILE.write_text(json.dumps(self.model, indent=1) + "\n")

    def run(self):
        a, n = self.a, self.index["plan"]["n"]
        k = 0
        while self.n_done() < n and k < len(self.U):
            rec = self.index["poses"].get(str(k))
            if rec is not None and rec["status"] in ("done", "unreachable", "skipped"):
                k += 1
                continue
            pose = pose_from(self.U[k], a, self.t)
            ok, why = self.t.check_sample(*pose["pair"])[:2]
            meta = {kk: v for kk, v in pose.items() if kk != "pair"}
            rec = self.index["poses"].setdefault(str(k), {"pose": meta, "status": "pending", "attempts": []})
            if not ok:
                rec["status"], rec["why"] = "unreachable", why
                self.save_index()
                print(f"pose {k}: unreachable ({why}), next")
                k += 1
                continue
            while True:
                attempt = len(rec["attempts"]) + 1
                off = predict(self.model, pose, a.offset)
                print(f"\npose {k} ({self.n_done()}/{n} done) try {attempt}: hole y {pose['grid_m'][0] * 100:+.1f} "
                      f"z {pose['grid_m'][1] * 100:+.1f} cm, height {pose['hole_height_m'] * 1000:+.1f} mm, tilt "
                      f"{pose['hole_tilt_deg'][0]:+.1f} {pose['hole_tilt_deg'][1]:+.1f} deg; offset "
                      f"{off[0]:+.2f} {off[1]:+.2f} mm")
                res = self.episode(k, pose, attempt, off)
                if res is None:
                    rec["attempts"].append({"attempt": attempt, "reason": "FAULT"})
                    self.save_index()
                    print("  reset FAULT -- check the robot. r: retry, s: skip pose, q: quit (teach window)")
                    self.key()
                    kk = None
                    while kk not in ("r", "s", "q"):
                        self.t.st.tick(None)
                        kk = self.key()
                    if kk == "r":
                        continue
                    if kk == "s":
                        rec["status"] = "skipped"
                        self.save_index()
                        break
                    raise Quit()
                good = res["reason"] == "SUCCESS" and (res["interventions"] == 0 or a.allow_interventions)
                name = f"pose_{k:03d}.npz" if good else f"pose_{k:03d}_try{attempt}.npz"
                path = self.dir / ("episodes" if good else "failed") / name
                self.save_episode(path, {**res, "pose": meta, "pose_index": k, "good": good})
                self.srv.buf = None
                res["file"] = str(path.relative_to(self.dir))
                rec["attempts"].append(res)
                print(f"  {res['reason']}, {res['interventions']} interventions, peak {res['push_back_peak']:.1f} N, "
                      f"return {res['return']:+.3f}, {res['frames']} frames -> {res['file']}")
                if good:
                    rec["status"] = "done"
                    self.save_index()
                    break
                self.save_index()
                kk = self.manual(k, pose, off)
                if kk == "s":
                    rec["status"] = "skipped"
                    self.save_index()
                    break
                if kk == "q":
                    raise Quit()
                if self.key() == "q":
                    raise Quit()
            if self.key() == "q":
                raise Quit()
            k += 1
        print(f"\n{self.n_done()} / {n} demos in {self.dir / 'episodes'}")


def main():
    ap = pht.build_parser(__doc__, default_out=str(HERE / "recordings" / "peg_hole_demos" / "teach"))
    ap.add_argument("--demo-dir", default=str(HERE / "recordings" / "peg_hole_demos" / "expert"))
    ap.add_argument("--n", type=int, default=100, help="demos to record")
    ap.add_argument("--sobol-seed", type=int, default=0)
    ap.add_argument("--extent", type=float, default=0.04, help="m, hole +- across its axis (the model's range)")
    ap.add_argument("--height", type=float, nargs=2, default=[-0.02, 0.0], help="m, hole along its axis")
    ap.add_argument("--tilt-range", type=float, default=3.0, help="deg, hole tilt about each cross axis")
    ap.add_argument("--hover", type=float, default=0.005, help="m above the rim the expert aligns at")
    ap.add_argument("--approach-speed", type=float, default=0.03, help="m/s to the hover pose")
    ap.add_argument("--insert-speed", type=float, default=None, help="m/s down the axis (default --speed)")
    ap.add_argument("--settle-ticks", type=int, default=5, help="ticks on the hover pose before inserting")
    ap.add_argument("--offset", choices=("model", "const"), default="model")
    ap.add_argument("--allow-interventions", action="store_true", help="count SUCCESS demos with interventions")
    ap.add_argument("--port-base", type=int, default=7611, help="Frames are published here too")
    ap.add_argument("--no-images", action="store_true")
    a = ap.parse_args()
    a.insert_speed = a.insert_speed if a.insert_speed is not None else a.speed
    if not a.allow_motion:
        ap.error("pass --allow-motion")
    pht.apply_setup(a)
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    io = pht.TeachIo(True)
    ex = SingleThreadedExecutor()
    ex.add_node(io)
    spin = threading.Thread(target=ex.spin, daemon=True)
    spin.start()
    t, lift_before, cams = None, None, None
    try:
        t0 = time.monotonic()
        while (srvmod.gateway_obs(io) is None or io.effort is None or io.ee("left") is None
               or io.ee("right") is None) and time.monotonic() - t0 < 15.0:
            time.sleep(0.1)
        if srvmod.gateway_obs(io) is None:
            print("no /joint_states or achieved EE poses in 15 s - is the teleop running?")
            return
        lift_now = dict(zip(io.raw_js[1], io.raw_js[2])).get("lift_joint")
        if lift_now is None or abs(lift_now - a.lift) > 0.003:
            print(f"lift is {lift_now}, not {a.lift:+.3f}: run peg_hole_safe_setup.py first; quitting")
            return
        if not a.no_images:
            cams = images.WristCameras()
            t0 = time.monotonic()
            while any(v is None for v in cams.grab()[0].values()) and time.monotonic() - t0 < 10.0:
                time.sleep(0.1)
            missing = [c for c, v in cams.grab()[0].items() if v is None]
            if missing:
                print(f"no wrist image from {missing} in 10 s (UDP {[images.PORTS[c] for c in missing]}; is the "
                      f"ffw_stream receiver holding the ports?) -- or pass --no-images; quitting")
                return
        if a.lock_lift:
            lift_before = io.set_joint_locked("lift_joint", True)
        t = pht.Teach(io, a)
        t.set_tool_model(a.tool_model)
        t.st.arms["left"] = phr.Arm("left", io.ee("left"), io.latest_site("left"))
        io.set_hold_side("left", True)
        t.take_right()
        srv = DemoServer(io, a.port_base, t)
        srv.cams = cams
        print(f"teach recording to {t.out}; demos to {a.demo_dir}; frames also on port {a.port_base}")
        Recorder(srv, a).run()
    except (KeyboardInterrupt, Quit, pht.Abort) as e:
        print(f"\nstopped ({type(e).__name__}); rerun to resume")
    finally:
        if cams is not None:
            cams.stop()
        if t is not None:
            print(f"session saved: {t.rec.save_session()}")
            t.summary.close()
        io.set_hold(False)
        print("both arms released to the SpaceMouse")
        if a.lock_lift and lift_before is False:
            io.set_joint_locked("lift_joint", False)
        time.sleep(0.2)
        ex.shutdown()
        spin.join(timeout=2.0)
        io.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
