#!/usr/bin/env python3
"""Compare three drive_to twist laws for one swerve base step (offline, no ROS).

Laws (head_cam_calib_capture.py drive_to):
  old     separate loops: v = gain*e_xy (cap vmax), wz = 1.25*gain*e_yaw (cap wmax)
  ramp    as old, wz * r, r = clamp(1 - |e_xy|/|e_xy0|, 0, 1)   (rotation after translation)
  common  one scale: k = min(gain, vmax/|e_xy|, wmax/|e_yaw|), twist = k*e   (current code)

Each runs through a model of ffw_swerve_drive_controller: per-module target
atan2(vy + wz*x, vx - wz*y), 180 deg flip when the turn exceeds 90 deg, steering
slew + settle error, wheel speed lag, and the alignment gate (a module with
|wheel| >= 0.1 rad/s misaligned past 1 rad, or a slower one past 0.01 rad, zeroes
ALL wheels; swerve_drive_controller.cpp ~1196-1263). Module offsets from the
follower controller yaml. Steer speed / settle error / wheel lag are guesses.

Window: per law, wheel headings (red = gated), base_link path with the goal
tolerance ring, steering angle vs time (red band = gated). Sliders re-run;
Play animates; the time slider scrubs.

    python3 swerve_step_sim.py
"""
import math

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import FuncFormatter
from matplotlib.widgets import Button, RadioButtons, Slider

MOD = np.array([[0.1371, 0.2554], [0.1371, -0.2554], [-0.2899, 0.0]])  # left, right, rear (x, y) [m]
R_WHEEL, DT, T_MAX = 0.0825, 0.005, 12.0
LAWS = [('old', 'Separate loops (before)', '#c27a12'),
        ('ramp', 'Rotation ramp (proposed)', '#5b5bd6'),
        ('common', 'Common scale (now)', '#0f8a7e')]
PRESETS = {
    'walk (5, 3 cm, 3 deg)': dict(dx=5, dy=3, dyaw=3, gain=4.0, tolm=3.0, told=1.5),
    'grid (10 cm, 0, 5 deg)': dict(dx=10, dy=0, dyaw=5, gain=1.2, tolm=1.0, told=0.5),
    'yaw-heavy (1.5 cm, 3 deg)': dict(dx=1.5, dy=0, dyaw=3, gain=4.0, tolm=3.0, told=1.5),
    'sideways (0, -5 cm, -3 deg)': dict(dx=0, dy=-5, dyaw=-3, gain=4.0, tolm=3.0, told=1.5),
}
VMAX, WMAX, PERIOD = 0.12, 0.20, 0.05
GATE_MOVE, GATE_SLOW, SLOW_SPD = 1.0, 0.01, 0.1

# least-squares body twist from module ground velocities: rows [1 0 -y], [0 1 x]
_A = np.vstack([[[1, 0, -y], [0, 1, x]] for x, y in MOD]).astype(float)
PINV = np.linalg.pinv(_A)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def twist_of(law, p, ex, ey, eyaw, e0):
    if law == 'common':
        k = min(p['gain'], VMAX / max(math.hypot(ex, ey), 1e-9), WMAX / max(abs(eyaw), 1e-9))
        return k * ex, k * ey, k * eyaw
    vx, vy = ex * p['gain'], ey * p['gain']
    n = math.hypot(vx, vy)
    if n > VMAX:
        vx, vy = vx * VMAX / n, vy * VMAX / n
    wz = max(-WMAX, min(WMAX, 1.25 * p['gain'] * eyaw))
    if law == 'ramp':
        wz *= min(1.0, max(0.0, 1.0 - math.hypot(ex, ey) / max(e0, 1e-9)))
    return vx, vy, wz


def simulate(law, p):
    gx, gy, gyaw = p['dx'] / 100, p['dy'] / 100, math.radians(p['dyaw'])
    tolm, told = p['tolm'] / 100, math.radians(p['told'])
    srate, dead, tau = math.radians(p['srate']) * DT, math.radians(p['sdead']), p['wtau'] / 1000
    x = y = th = t = 0.0
    next_cmd, cmd, done, stall = 0.0, (0.0, 0.0, 0.0), None, 0.0
    steer, omega, travel = np.zeros(3), np.zeros(3), np.zeros(3)
    e0 = math.hypot(gx, gy)
    fr = []
    k = 0
    while t < T_MAX:
        if t >= next_cmd - 1e-9:
            next_cmd += PERIOD
            exw, eyw, eyaw = gx - x, gy - y, wrap(gyaw - th)
            if math.hypot(exw, eyw) < tolm and abs(eyaw) < told:
                cmd = (0.0, 0.0, 0.0)
                if done is None:
                    done = t
            else:
                c, s = math.cos(th), math.sin(th)
                cmd = twist_of(law, p, c * exw + s * eyw, -s * exw + c * eyw, eyaw, e0)
        zero = cmd == (0.0, 0.0, 0.0)
        aligned, wcmd, tgts = True, np.zeros(3), steer.copy()
        if not zero:
            for i, (mx, my) in enumerate(MOD):
                wvx, wvy = cmd[0] - cmd[2] * my, cmd[1] + cmd[2] * mx
                tgt, d = math.atan2(wvy, wvx + 1e-9), 1.0
                if abs(wrap(tgt - steer[i])) > math.pi / 2:
                    tgt, d = wrap(tgt + math.pi), -1.0
                tgts[i] = tgt
                err = abs(wrap(tgt - steer[i]))
                if (err >= GATE_MOVE) if abs(omega[i]) >= SLOW_SPD else (err >= GATE_SLOW):
                    aligned = False
                wcmd[i] = d * math.hypot(wvx, wvy) / R_WHEEL
        if not zero and not aligned:
            stall += DT
        for i in range(3):
            d = wrap(tgts[i] - steer[i])
            step = max(-srate, min(srate, d))
            if abs(d) <= dead:
                step = 0.0
            elif abs(d - step) < dead:
                step = d - math.copysign(dead * 0.5, d)
            steer[i] = wrap(steer[i] + step)
            travel[i] += abs(step)
            target = 0.0 if (zero or not aligned) else wcmd[i]
            omega[i] += (target - omega[i]) * min(1.0, DT / max(tau, 1e-6))
        g = np.empty(6)
        g[0::2] = omega * R_WHEEL * np.cos(steer)
        g[1::2] = omega * R_WHEEL * np.sin(steer)
        vx, vy, wz = PINV @ g
        c, s = math.cos(th), math.sin(th)
        x += (c * vx - s * vy) * DT
        y += (s * vx + c * vy) * DT
        th += wz * DT
        if k % 4 == 0:
            fr.append((t, x, y, th, *steer, *omega, *tgts, (not zero) and (not aligned)))
        k += 1
        t += DT
        if done is not None and t > done + 0.25:
            break
    return dict(f=np.array(fr, dtype=float), done=done, stall=stall,
                travel=math.degrees(travel.max()), goal=(gx, gy, gyaw), tolm=tolm)


# frame columns
T, X, Y, TH = 0, 1, 2, 3
ST, OM, TG, BL = slice(4, 7), slice(7, 10), slice(10, 13), 13


class App:
    def __init__(self):
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('Swerve step comparison')
        outer = self.fig.add_gridspec(2, 3, left=0.04, right=0.985, top=0.88, bottom=0.30,
                                      hspace=0.32, wspace=0.16, height_ratios=[1.25, 1])
        self.ax = []
        for c in range(3):
            top = outer[0, c].subgridspec(1, 2, wspace=0.08)
            self.ax.append([self.fig.add_subplot(top[0, 0]), self.fig.add_subplot(top[0, 1]),
                            self.fig.add_subplot(outer[1, c])])
        self.p = dict(PRESETS['walk (5, 3 cm, 3 deg)'], srate=180.0, sdead=0.2, wtau=80.0)
        self.t_now, self.playing, self.res = 0.0, False, None
        self._build_widgets()
        self.run()
        self.timer = self.fig.canvas.new_timer(interval=40)
        self.timer.add_callback(self._tick)
        self.timer.start()

    def _build_widgets(self):
        f = self.fig
        specs = [('dx', 'dx [cm]', -15, 15), ('dy', 'dy [cm]', -15, 15), ('dyaw', 'dyaw [deg]', -10, 10),
                 ('gain', 'gain', 0.5, 6), ('tolm', 'tol [cm]', 0.3, 5), ('told', 'tol [deg]', 0.2, 3),
                 ('srate', 'steer speed [deg/s]', 30, 720), ('sdead', 'steer settle err [deg]', 0, 1.0),
                 ('wtau', 'wheel lag tau [ms]', 10, 300)]
        self.sl = {}
        for i, (k, lab, lo, hi) in enumerate(specs):
            col, row = i // 3, i % 3
            ax = f.add_axes([0.10 + col * 0.25, 0.19 - row * 0.04, 0.11, 0.022])
            s = Slider(ax, lab, lo, hi, valinit=self.p[k])
            s.on_changed(lambda v, k=k: self._set(k, v))
            self.sl[k] = s
        axr = f.add_axes([0.80, 0.09, 0.18, 0.15])
        axr.set_title('preset', fontsize=9)
        self.radio = RadioButtons(axr, list(PRESETS))
        self.radio.on_clicked(self._preset)
        axt = f.add_axes([0.08, 0.04, 0.62, 0.025])
        self.tsl = Slider(axt, 'time [s]', 0, 1, valinit=1)
        self.tsl.on_changed(self._scrub)
        axb = f.add_axes([0.80, 0.03, 0.08, 0.045])
        self.btn = Button(axb, 'Play')
        self.btn.on_clicked(self._play)
        self._mute = False

    def _set(self, k, v):
        if self._mute:
            return
        self.p[k] = float(v)
        self.run()

    def _preset(self, name):
        self._mute = True
        for k, v in PRESETS[name].items():
            self.p[k] = v
            self.sl[k].set_val(v)
        self._mute = False
        self.run()

    def _scrub(self, v):
        if self.res is None or self._mute:
            return
        self.t_now = v * self.t_max
        self.draw()

    def _play(self, _):
        self.playing = not self.playing
        if self.playing and self.t_now >= self.t_max - 1e-6:
            self.t_now = 0.0
        self.btn.label.set_text('Pause' if self.playing else 'Play')

    def _tick(self):
        if not self.playing:
            return
        self.t_now = min(self.t_max, self.t_now + 0.02)    # half real time
        self._mute = True
        self.tsl.set_val(self.t_now / self.t_max)
        self._mute = False
        self.draw()
        if self.t_now >= self.t_max:
            self.playing = False
            self.btn.label.set_text('Play')

    def run(self):
        self.res = [simulate(law, self.p) for law, _, _ in LAWS]
        self.t_max = max(r['f'][-1, T] for r in self.res)
        self.t_now = self.t_max
        a = np.degrees(np.hstack([r['f'][:, 4:7].ravel() for r in self.res] + [r['f'][:, 10:13].ravel() for r in self.res]))
        pad = max(5.0, 0.1 * np.ptp(a))
        self.ylim = (a.min() - pad, a.max() + pad)
        self.draw()

    def draw(self):
        times = [r['done'] if r['done'] is not None else math.inf for r in self.res]
        best = min(times)
        for c, ((law, name, col), r) in enumerate(zip(LAWS, self.res)):
            f = r['f']
            i = max(0, np.searchsorted(f[:, T], self.t_now, side='right') - 1)
            q = f[i]
            aw, ap, aa = self.ax[c]
            done = f'{r["done"]:.2f} s' if r['done'] is not None else f'> {T_MAX:.0f} s'
            tag = '  (fastest)' if times[c] == best else ''
            # ── wheels ──
            aw.clear()
            aw.text(0.0, 1.28, name, transform=aw.transAxes, fontsize=11, fontweight='bold', color=col)
            aw.text(0.0, 1.13, f'time to tol {done}{tag}   gated {r["stall"]:.2f} s   steer travel {r["travel"]:.0f}°',
                    transform=aw.transAxes, fontsize=9, color='0.15')
            tri = np.vstack([MOD, MOD[:1]])
            aw.plot(-tri[:, 1], tri[:, 0], color='0.85', lw=1.5)
            aw.plot([0, 0], [0, 0.08], color='0.5', lw=1)
            aw.text(0, 0.1, 'front', ha='center', fontsize=7, color='0.4')
            gated = bool(q[BL])
            for m, (mx, my) in enumerate(MOD):
                st, tg, om = q[4 + m], q[10 + m], q[7 + m]
                u = np.array([-math.sin(st), math.cos(st)])        # plot coords: (-y, x)
                ut = np.array([-math.sin(tg), math.cos(tg)])
                cx, cy = -my, mx
                aw.plot([cx - ut[0] * 0.07, cx + ut[0] * 0.07], [cy - ut[1] * 0.07, cy + ut[1] * 0.07],
                        ls='--', color='0.55', lw=1)
                aw.plot([cx - u[0] * 0.045, cx + u[0] * 0.045], [cy - u[1] * 0.045, cy + u[1] * 0.045],
                        color='#d64545' if gated else '0.1', lw=6, solid_capstyle='round')
                sp = om * R_WHEEL
                if abs(sp) > 0.002:
                    L = min(0.16, abs(sp) * 1.6)
                    sg = math.copysign(1, sp)
                    aw.annotate('', xy=(cx + sg * u[0] * (0.05 + L), cy + sg * u[1] * (0.05 + L)),
                                xytext=(cx + sg * u[0] * 0.05, cy + sg * u[1] * 0.05),
                                arrowprops=dict(arrowstyle='->', color=col, lw=1.8))
                aw.text(cx, cy + (0.08 if mx > 0 else -0.1), f'{math.degrees(st):.0f}°', ha='center', fontsize=8, color='0.3')
            aw.set_title('GATED: all wheels held at 0' if gated else 'wheels (dashed = target)',
                         fontsize=8, color='#d64545' if gated else '0.4', loc='left')
            aw.set_xlim(-0.45, 0.45)
            aw.set_ylim(-0.42, 0.32)
            aw.set_aspect('equal')
            aw.set_xticks([])
            aw.set_yticks([])
            # ── path ──
            ap.clear()
            gx, gy, gyaw = r['goal']
            ap.add_patch(plt.Circle((-gy * 100, gx * 100), r['tolm'] * 100, fill=False, ls='--', color='0.5'))
            ap.plot([-gy * 100], [gx * 100], 'o', color='0.4', ms=4)
            ap.plot([0], [0], 'o', color='0.4', ms=3)
            ap.text(0.3, -0.3, 'start', fontsize=7, color='0.4')
            seen = f[: i + 1]
            ap.plot(-seen[:, Y] * 100, seen[:, X] * 100, color=col, lw=2)
            ap.plot([-q[Y] * 100], [q[X] * 100], 'o', color=col, ms=6)
            ap.plot([-q[Y] * 100, -q[Y] * 100 - math.sin(q[TH]) * 1.2], [q[X] * 100, q[X] * 100 + math.cos(q[TH]) * 1.2],
                    color=col, lw=1.5)
            ext = max(abs(gx) * 100 / 2 + r['tolm'] * 100, abs(gy) * 100 / 2 + r['tolm'] * 100, 2) * 1.15
            ap.set_xlim(-gy * 50 - ext, -gy * 50 + ext)
            ap.set_ylim(gx * 50 - ext, gx * 50 + ext)
            ap.set_aspect('equal')
            ap.grid(True, color='0.92')
            ap.tick_params(labelsize=7)
            ap.set_xlabel('y [cm] (+ = left)', fontsize=7)
            ap.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{-v + 0.0:g}'))
            ap.set_ylabel('x [cm] (forward)', fontsize=7)
            ap.text(0.03, 0.03, f'x {q[X] * 100:.1f}  y {q[Y] * 100:.1f} cm\nyaw {math.degrees(q[TH]):.2f}° → {math.degrees(gyaw):.1f}°',
                    transform=ap.transAxes, fontsize=7, color='0.25', va='bottom')
            # ── steering angle vs time ──
            aa.clear()
            t = f[:, T]
            bl = f[:, BL] > 0.5
            if bl.any():
                aa.fill_between(t, self.ylim[0], self.ylim[1], where=bl, color='#d64545', alpha=0.18, step='post', lw=0)
            for m, alpha in zip(range(3), (1.0, 0.65, 0.4)):
                aa.plot(t, np.degrees(f[:, 10 + m]), ls='--', color=col, alpha=alpha, lw=1)
                aa.plot(t, np.degrees(f[:, 4 + m]), color=col, alpha=alpha, lw=2, label=('left', 'right', 'rear')[m])
            if r['done'] is not None:
                aa.axvline(r['done'], color='0.5', ls=':', lw=1)
            aa.axvline(self.t_now, color='0.1', lw=1)
            aa.set_xlim(0, self.t_max)
            aa.set_ylim(*self.ylim)
            aa.grid(True, color='0.92')
            aa.tick_params(labelsize=7)
            aa.set_xlabel('time [s]', fontsize=7)
            aa.set_ylabel('steering [deg]', fontsize=7)
            aa.legend(fontsize=6, loc='lower right', ncol=3)
        self.fig.suptitle('drive_to step through a swerve controller model   ·   red = alignment gate holding every wheel at 0   ·   '
                          'dashed = target', fontsize=10, y=0.985)
        self.fig.canvas.draw_idle()


if __name__ == '__main__':
    app = App()
    plt.show()
