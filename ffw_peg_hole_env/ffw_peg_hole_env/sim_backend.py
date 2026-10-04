"""MuJoCo backend for the peg-in-hole scene (scene_zero_margin.xml).

Lockstep: tick() advances physics by exactly one agent period (1/hz of sim
time) and returns immediately, so the sim runs as fast as the CPU allows.

Arms are commanded by **EE pose** -- the IK end-effector site
(left/right_gripper_site), the same point the real IK solver tracks and the
gateway reports in Obs -- so a rotation delta turns about the same point on
both backends. The peg/hole tool sites (right_peg_site, left_hole_site) hang
off the EE by fixed transforms (ee_to_tool). Each tick solves a
damped-least-squares IK from the previous joint targets, and the position
servos get MuJoCo's own gravity compensation (gravcomp through the
actuators) -- without it the kp = 3000 servos sag 4-7 mm.
"""
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from .geometry import T_from

DEFAULT_SCENE = (Path(__file__).resolve().parents[2] / "ffw_collision_checker" / "3rd_party"
                 / "robotis_ffw" / "scene_zero_margin.xml")
SIDES = ("left", "right")
ARM_JOINTS = {s: [f"arm_{s[0]}_joint{j}" for j in range(1, 8)] for s in SIDES}
TOOL_SITE = {"left": "left_hole_site", "right": "right_peg_site"}
EE_SITE = {"left": "left_gripper_site", "right": "right_gripper_site"}
# Gripper joint angles held fixed for the whole run (the model welds the
# fingers): the robot's /joint_states on 2026-10-01, peg and hole gripped.
DEFAULT_GRIPPER = {"right": 0.97868, "left": 0.98021}
GRIPPER_MAX = 1.125                       # gripper_*_joint1 range in ffw_bg2.xml
# Finger bodies all mimic gripper_<s>_joint1 about their own x (ffw_bg2.xml).
FINGER_AXIS_SIGN = {"r1": 1.0, "r2": -1.0, "l1": -1.0, "l2": 1.0}
# Wrist camera mounts, in the gripper (arm_*_link7) frame: "up" = link7 +x (the
# insertion axis, up out of the hole), "forward" = link7 -z (out of the
# gripper). Offsets [m] from the CAD camera position, extra downward pitch of the
# view [deg]. Image up = link7 +x on both cameras.
DEFAULT_CAM_MOUNT = {
    "right": {"up": 0.10, "forward": 0.05, "pitch_down_deg": 30.0},   # peg camera
    "left": {"up": 0.0, "forward": 0.0, "pitch_down_deg": 0.0},       # hole camera
}
# Robot joint angles on 2026-10-01: left near a peg-hole test pose, right parked.
DEFAULT_START = {
    "arm_l_joint1": 0.0435, "arm_l_joint2": 0.96202, "arm_l_joint3": -0.70616, "arm_l_joint4": -1.26046,
    "arm_l_joint5": 0.97725, "arm_l_joint6": -0.76294, "arm_l_joint7": -0.44259,
    "arm_r_joint1": -0.47383, "arm_r_joint2": 0.0, "arm_r_joint3": 1.18344, "arm_r_joint4": -1.14445,
    "arm_r_joint5": -0.52614, "arm_r_joint6": -0.17487, "arm_r_joint7": -0.08441,
    "head_joint1": 0.6412, "head_joint2": 0.05522, "lift_joint": 0.0,
}
DEFAULT_LIFT = -0.30   # m, lift_joint (0 = top); the user's choice, hole centre re-surveyed for it (peg_hole.py)


class MujocoBackend:
    def __init__(self, hz=15.0, scene=DEFAULT_SCENE, start=None, view=False, timestep=None,
                 cameras=True, cam_res=(128, 128), cam_fovy=58.0, gripper=None, cam_mount=None, lift=DEFAULT_LIFT,
                 lift_free=False, lift_weight=10.0):
        self.gripper = dict(DEFAULT_GRIPPER if gripper is None else gripper)
        self.cam_mount = {**DEFAULT_CAM_MOUNT, **(cam_mount or {})}
        self.m = self._build_model(scene, timestep, cameras, cam_fovy, self.gripper, self.cam_mount)
        self.d = mujoco.MjData(self.m)
        self.cam_res = cam_res
        self.renderer = None
        self.cam_names = [f"wrist_cam_{s[0]}" for s in SIDES] if cameras else []
        self.kin = mujoco.MjData(self.m)        # scratch data for IK (kinematics only)
        self.hz = hz
        self.nsub = max(1, int(round(1.0 / hz / self.m.opt.timestep)))
        m = self.m
        self.jid = {s: [m.joint(n).id for n in ARM_JOINTS[s]] for s in SIDES}
        self.qadr = {s: np.array([m.jnt_qposadr[j] for j in self.jid[s]]) for s in SIDES}
        self.dadr = {s: np.array([m.jnt_dofadr[j] for j in self.jid[s]]) for s in SIDES}
        joint_to_act = {m.joint(m.actuator_trnid[a][0]).name: a for a in range(m.nu)}
        self.aid = {s: np.array([joint_to_act[n] for n in ARM_JOINTS[s]]) for s in SIDES}
        self.kp = {s: m.actuator_gainprm[self.aid[s], 0].copy() for s in SIDES}
        self.lo = {s: np.array([m.jnt_range[j][0] if m.jnt_limited[j] else -np.inf for j in self.jid[s]]) for s in SIDES}
        self.hi = {s: np.array([m.jnt_range[j][1] if m.jnt_limited[j] else np.inf for j in self.jid[s]]) for s in SIDES}
        self.tool_sid = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, TOOL_SITE[s]) for s in SIDES}
        self.sid = {s: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, EE_SITE[s]) for s in SIDES}
        self.peg_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "peg")
        self.wall_geoms = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"hole_wall_{n}")
                           for n in ("py", "ny", "pz", "nz")]
        self._jp = np.zeros((3, m.nv))
        self._jr = np.zeros((3, m.nv))
        # lift_free=False: the lift is never commanded after this; its position actuator holds it (= the
        # solver's hard lock). lift_free=True: command_all() solves both arms + the lift together, like the
        # real IK solver with the lift unlocked; lift_weight makes the lift costlier than an arm joint.
        self.lift, self.lift_free, self.lift_weight = lift, lift_free, lift_weight
        jl = m.joint("lift_joint").id
        self.lift_qadr, self.lift_dadr = m.jnt_qposadr[jl], m.jnt_dofadr[jl]
        self.lift_aid = joint_to_act["lift_joint"]
        self.lift_lo, self.lift_hi = m.jnt_range[jl]
        self.lift_cmd = lift
        self.set_joints({**(DEFAULT_START if start is None else start), "lift_joint": lift})
        # fixed EE -> tool transform per arm (both sites live on arm_*_link7)
        self.ee_to_tool = {s: np.linalg.inv(self.site_pose(s)) @ self.tool_pose(s) for s in SIDES}
        self.viewer = None
        if view:
            from mujoco import viewer as mj_viewer
            self.viewer = mj_viewer.launch_passive(m, self.d)

    @staticmethod
    def _build_model(scene, timestep, cameras, cam_fovy, gripper, cam_mount):
        """Load the scene and add, without touching the MJCF files:
          * gravity compensation inside MuJoCo for the arm links, routed through
            the arm actuators (so actuator force still contains the gravity load,
            like the robot's current), replacing a per-substep Python feed-forward
          * wrist cameras on camera_l_link / camera_r_link looking along the
            link +x (REP-103 camera link), D405-like vertical FOV
          * optionally a different physics timestep (implicitfast integrator)
          * the welded finger bodies rotated to the fixed gripper angles
          * collision primitives moved to geom group 3 (not drawn by default, so
            cameras see only the visual meshes); peg and hole made opaque"""
        spec = mujoco.MjSpec.from_file(str(scene))
        for g in spec.geoms:
            if g.name == "peg" or g.name.startswith("hole_wall_"):
                g.rgba = [g.rgba[0], g.rgba[1], g.rgba[2], 1.0]   # task parts opaque
            elif (g.contype or g.conaffinity) and g.type != mujoco.mjtGeom.mjGEOM_PLANE:   # keep the floor
                g.group = 3        # collision primitives: hidden in cameras and the viewer by default
        for side, q in gripper.items():
            for finger, sign in FINGER_AXIS_SIGN.items():
                b = spec.body(f"gripper_{side[0]}_rh_p12_rn_{finger}")
                r = np.zeros(4)
                mujoco.mju_axisAngle2Quat(r, np.array([1.0, 0.0, 0.0]), sign * q)
                out = np.zeros(4)
                mujoco.mju_mulQuat(out, np.array(b.quat, dtype=float), r)
                b.quat = out
        for b in spec.bodies:
            if b.name.startswith(("arm_l_", "arm_r_", "gripper_", "camera_", "peg_box")):
                b.gravcomp = 1.0
        # everything the lift carries is weightless too, so the lift's position actuator holds
        # its height exactly (it sagged 9 mm at -0.15 m): with the solver-side lock, a hard lock
        stack = [spec.joint("lift_joint").parent]
        while stack:
            b = stack.pop()
            b.gravcomp = 1.0
            stack.extend(b.bodies)
        for j in spec.joints:
            if j.name.startswith(("arm_l_joint", "arm_r_joint")):
                j.actgravcomp = True
        if cameras:
            for side in ("l", "r"):
                mount = cam_mount["left" if side == "l" else "right"]
                cl = spec.body(f"camera_{side}_link")             # CAD D405 pose in arm_*_link7
                p_c = np.array(cl.pos, dtype=float)
                R_c = np.zeros(9)
                mujoco.mju_quat2Mat(R_c, np.array(cl.quat, dtype=float))
                view0 = R_c.reshape(3, 3)[:, 0]                   # CAD view direction (camera link +x)
                x_l7 = np.array([1.0, 0.0, 0.0])
                up0 = x_l7 - (x_l7 @ view0) * view0               # image up: link7 +x, orthogonal to the view
                up0 /= np.linalg.norm(up0)
                p = np.radians(mount["pitch_down_deg"])
                view = np.cos(p) * view0 - np.sin(p) * up0       # pitch the view down
                up = np.sin(p) * view0 + np.cos(p) * up0
                right = np.cross(view, up)
                cam = spec.body(f"arm_{side}_link7").add_camera()
                cam.name = f"wrist_cam_{side}"
                cam.pos = list(p_c + mount["up"] * x_l7 + mount["forward"] * np.array([0.0, 0.0, -1.0]))
                cam.alt.xyaxes = [*right, *up]                    # MuJoCo camera: x right, y up, looks along -z
                cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
                cam.fovy = cam_fovy
        if timestep is not None:
            spec.option.timestep = timestep
            spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        return spec.compile()

    def render(self):
        """Wrist images {"right": HxWx3 uint8, "left": ...} from the current state."""
        if not self.cam_names:
            return {}
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.m, self.cam_res[1], self.cam_res[0])
            self._vopt = mujoco.MjvOption()
            self._vopt.sitegroup[:] = 0                    # no debug sites in the policy's images
        out = {}
        for side in SIDES:
            self.renderer.update_scene(self.d, camera=f"wrist_cam_{side[0]}", scene_option=self._vopt)
            out[side] = self.renderer.render().copy()
        return out

    # --- state --------------------------------------------------------------
    def set_joints(self, q_by_name):
        m, d = self.m, self.d
        for j in range(m.njnt):
            name = m.joint(j).name
            if name in q_by_name:
                d.qpos[m.jnt_qposadr[j]] = q_by_name[name]
        d.qvel[:] = 0.0
        for a in range(m.nu):
            d.ctrl[a] = d.qpos[m.jnt_qposadr[m.actuator_trnid[a][0]]]
        mujoco.mj_forward(m, d)
        self.qcmd = {s: d.qpos[self.qadr[s]].copy() for s in SIDES}

    def site_pose(self, side, data=None):
        """EE (IK end-effector site) pose in base_link."""
        data = self.d if data is None else data
        s = self.sid[side]
        return T_from(data.site_xpos[s].copy(), data.site_xmat[s].reshape(3, 3).copy())

    def tool_pose(self, side, data=None):
        """Peg (right) / hole (left) tool-site pose in base_link."""
        data = self.d if data is None else data
        s = self.tool_sid[side]
        return T_from(data.site_xpos[s].copy(), data.site_xmat[s].reshape(3, 3).copy())

    def gripper_normalized(self):
        """(right, left) in the Obs convention, 0.0 open .. 1.0 closed."""
        return (self.gripper["right"] / GRIPPER_MAX, self.gripper["left"] / GRIPPER_MAX)

    def all_joints(self):
        """(names, pos, vel, actuator torque) for every 1-dof joint in the model."""
        m, d = self.m, self.d
        names = [m.joint(j).name for j in range(m.njnt)]
        pos_ = np.array([d.qpos[m.jnt_qposadr[j]] for j in range(m.njnt)])
        vel = np.array([d.qvel[m.jnt_dofadr[j]] for j in range(m.njnt)])
        tau = np.zeros(m.njnt)
        for a in range(m.nu):
            tau[m.actuator_trnid[a][0]] = d.actuator_force[a]
        return names, pos_, vel, tau

    def joint_state(self, side):
        d = self.d
        return (d.qpos[self.qadr[side]].copy(), d.qvel[self.dadr[side]].copy(),
                d.actuator_force[self.aid[side]].copy())

    # --- IK -----------------------------------------------------------------
    def ik(self, side, T, q0, iters=6, damping=1e-4, tol=(1e-5, 1e-4)):
        """DLS IK for an EE-site pose; returns (q, pos_err, rot_err)."""
        m, k = self.m, self.kin
        k.qpos[:] = self.d.qpos
        q = q0.copy()
        sid = self.sid[side]
        err = np.zeros(6)
        for _ in range(iters):
            k.qpos[self.qadr[side]] = q
            mujoco.mj_kinematics(m, k)
            mujoco.mj_comPos(m, k)
            R = k.site_xmat[sid].reshape(3, 3)
            err[:3] = T[:3, 3] - k.site_xpos[sid]
            err[3:] = Rot.from_matrix(T[:3, :3] @ R.T).as_rotvec()
            if np.linalg.norm(err[:3]) < tol[0] and np.linalg.norm(err[3:]) < tol[1]:
                break
            mujoco.mj_jacSite(m, k, self._jp, self._jr, sid)
            J = np.vstack([self._jp, self._jr])[:, self.dadr[side]]
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + damping * np.eye(6), err), self.lo[side], self.hi[side])
        return q, float(np.linalg.norm(err[:3])), float(np.linalg.norm(err[3:]))

    def reachable(self, side, T, tol=(0.0005, np.radians(0.2)), q0=None):
        """Can this side's EE site reach pose T (IK to convergence)?"""
        _, pe, re = self.ik(side, T, self.qcmd[side] if q0 is None else q0, iters=300)
        return pe < tol[0] and re < tol[1]

    def command(self, side, T):
        """Set this side's joint targets from an EE pose (warm-started IK)."""
        self.qcmd[side], _, _ = self.ik(side, T, self.qcmd[side])

    def command_all(self, targets, iters=6, damping=1e-4):
        """Both EE targets. Lift locked: one IK per arm. Lift free: one weighted DLS
        over arm_l (7) + arm_r (7) + lift, both sites' errors stacked."""
        if not self.lift_free:
            for side, T in targets.items():
                self.command(side, T)
            return
        m, k = self.m, self.kin
        k.qpos[:] = self.d.qpos
        cols = np.concatenate([self.dadr["left"], self.dadr["right"], [self.lift_dadr]])
        winv = np.ones(15)
        winv[-1] = 1.0 / self.lift_weight
        q = {s: self.qcmd[s].copy() for s in SIDES}
        lift = self.lift_cmd
        e = np.zeros(12)
        for _ in range(iters):
            for s in SIDES:
                k.qpos[self.qadr[s]] = q[s]
            k.qpos[self.lift_qadr] = lift
            mujoco.mj_kinematics(m, k)
            mujoco.mj_comPos(m, k)
            J = np.zeros((12, 15))
            for i, s in enumerate(SIDES):
                sid, T = self.sid[s], targets[s]
                e[6 * i:6 * i + 3] = T[:3, 3] - k.site_xpos[sid]
                e[6 * i + 3:6 * i + 6] = Rot.from_matrix(T[:3, :3] @ k.site_xmat[sid].reshape(3, 3).T).as_rotvec()
                mujoco.mj_jacSite(m, k, self._jp, self._jr, sid)
                J[6 * i:6 * i + 6] = np.vstack([self._jp, self._jr])[:, cols]
            dq = winv * (J.T @ np.linalg.solve((J * winv) @ J.T + damping * np.eye(12), e))
            for i, s in enumerate(SIDES):
                q[s] = np.clip(q[s] + dq[7 * i:7 * i + 7], self.lo[s], self.hi[s])
            lift = float(np.clip(lift + dq[14], self.lift_lo, self.lift_hi))
        self.qcmd = q
        self.lift_cmd = lift

    # --- time -----------------------------------------------------------------
    def tick(self):
        """Advance one agent period of sim time (no wall-clock wait)."""
        m, d = self.m, self.d
        for s in SIDES:
            d.ctrl[self.aid[s]] = self.qcmd[s]          # gravity compensation is inside MuJoCo
        d.ctrl[self.lift_aid] = self.lift_cmd
        mujoco.mj_step(m, d, nstep=self.nsub)
        if self.viewer is not None:
            self.viewer.sync()

    def teleport(self, targets, iters=200, settle_ticks=8):
        """Place both EE sites at target poses directly (reset only): solve IK
        to convergence, write qpos, zero velocities, settle. Returns the worst
        (pos_err, rot_err) after settling. The lift goes back to its nominal height."""
        self.d.qpos[self.lift_qadr] = self.lift_cmd = self.lift
        for side, T in targets.items():
            q, _, _ = self.ik(side, T, self.qcmd[side], iters=iters)
            self.d.qpos[self.qadr[side]] = q
            self.qcmd[side] = q
        self.d.qvel[:] = 0.0
        mujoco.mj_forward(self.m, self.d)
        for _ in range(settle_ticks):
            self.tick()
        errs = [self._err(self.site_pose(s), T) for s, T in targets.items()]
        return max(e[0] for e in errs), max(e[1] for e in errs)

    @staticmethod
    def _err(A, B):
        return (float(np.linalg.norm(A[:3, 3] - B[:3, 3])),
                float(np.linalg.norm(Rot.from_matrix(B[:3, :3] @ A[:3, :3].T).as_rotvec())))

    # --- contact ----------------------------------------------------------------
    def peg_hole_force(self):
        """Total normal contact force between the peg and the hole walls [N]."""
        m, d = self.m, self.d
        f6 = np.zeros(6)
        total = 0.0
        walls = set(self.wall_geoms)
        for i in range(d.ncon):
            c = d.contact[i]
            if (c.geom1 == self.peg_geom and c.geom2 in walls) or (c.geom2 == self.peg_geom and c.geom1 in walls):
                mujoco.mj_contactForce(m, d, i, f6)
                total += abs(f6[0])
        return total

    def tool_geometry(self):
        """(peg tip along the peg site x, hole rim along the hole site x) [m], both
        fixed by the model: the lowest peg corner and the top of the walls."""
        m, d = self.m, self.d
        T_peg = self.tool_pose("right")
        Tg = T_from(d.geom_xpos[self.peg_geom], d.geom_xmat[self.peg_geom].reshape(3, 3))
        hs = m.geom_size[self.peg_geom]
        corners = np.array([[sx * hs[0], sy * hs[1], sz * hs[2], 1.0]
                            for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        tip = float(min((np.linalg.inv(T_peg) @ Tg @ corners.T)[0]))
        T_hole = self.tool_pose("left")
        rim = max(float((np.linalg.inv(T_hole) @ T_from(d.geom_xpos[w], d.geom_xmat[w].reshape(3, 3))
                         @ np.array([m.geom_size[w][0], 0.0, 0.0, 1.0]))[0]) for w in self.wall_geoms)
        return tip, rim

    def close(self):
        """Free the offscreen renderer; with a viewer, wait until the user closes
        the window (closing the passive viewer from code segfaults, mujoco 3.x)."""
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None
        if self.viewer is not None:
            import time
            while self.viewer.is_running():
                time.sleep(0.1)
            self.viewer = None
