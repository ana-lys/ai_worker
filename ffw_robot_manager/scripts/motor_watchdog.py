#!/usr/bin/env python3
"""Passive motor-event watchdog for the FFW follower.

Subscribes only -- it never calls get_dxl_data / set_dxl_data (each of those
blocks the driver's read/write loop for up to 1 s on a live robot), so it can
run alongside teleop without disturbing anything.

Watches:
  /joint_states             per watched joint: current (effort = Present
                            Current, raw counts) vs its Current Limit --
                            NEAR (>= --warn-frac), SATURATED (>= --sat-frac
                            held for --sat-hold s), TORQUE-LOSS heuristic
                            (|current| collapses to ~0 after holding load);
                            plus /joint_states stalls (gap > --stall-s)
  /ffw_{follower,base,sensor}/dxl_state
                            Hardware Error Status per ID (decoded bits) and
                            the bus comm_state, on every change
  /rosout                   WARN+ lines from the driver / controller_manager /
                            robot manager, collapsed per message kind
  /ai_worker/battery/*/state  voltage sag below --batt-min-v

Every event goes to the console and to a CSV (--csv). A status line with the
watched joints' position/current min..max is printed every --status-s.

  python3 motor_watchdog.py                       # right gripper (XH540, ID 130)
  python3 motor_watchdog.py --joint gripper_r_joint1:750 --joint gripper_l_joint1:0
                                                  # 0 = no limit known, just log
"""
import argparse
import csv
import math
import os
import re
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import BatteryState, JointState
from dynamixel_interfaces.msg import DynamixelState

# X-series Hardware Error Status bits (ROBOTIS e-Manual)
HW_BITS = {0: "InputVoltage", 2: "Overheating", 3: "MotorEncoder",
           4: "ElectricalShock", 5: "Overload"}
DXL_TOPICS = ("/ffw_follower/dxl_state", "/ffw_base/dxl_state", "/ffw_sensor/dxl_state")
ROSOUT_NODES = ("dynamixel_hardware_interface", "controller_manager", "ffw_robot_manager",
                "ros2_control_node", "joint_state_broadcaster")
MA_PER_RAW = 2.69  # X-series Present Current unit


def decode_hw(v):
    if not v:
        return "OK"
    names = [n for b, n in HW_BITS.items() if v & (1 << b)]
    other = v & ~sum(1 << b for b in HW_BITS)
    if other:
        names.append(f"bits0x{other:02x}")
    return "|".join(names)


class Watchdog(Node):
    def __init__(self, a):
        super().__init__("motor_watchdog")
        self.a = a
        self.joints = {}  # name -> limit (raw), 0 = unknown
        for spec in a.joint:
            name, _, lim = spec.partition(":")
            self.joints[name] = float(lim or 0)
        self.t0 = time.monotonic()
        self.last_js = None
        self.stalled = False
        self.st = {j: dict(near=False, sat_since=None, sat=False, loaded_since=None,
                           lost=False, pmin=math.inf, pmax=-math.inf,
                           cmin=math.inf, cmax=-math.inf, n=0) for j in self.joints}
        self.hw = {}       # (topic, id) -> last hw state
        self.comm = {}     # topic -> last comm_state
        self.rosout = {}   # kind -> [count, first, last]
        self.batt_low = {}

        os.makedirs(os.path.dirname(os.path.abspath(a.csv)), exist_ok=True)
        self.csvf = open(a.csv, "a", newline="")
        self.csvw = csv.writer(self.csvf)
        if self.csvf.tell() == 0:
            self.csvw.writerow(["wall_time", "t_s", "kind", "source", "detail"])

        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        for t in DXL_TOPICS:
            self.create_subscription(DynamixelState, t, lambda m, t=t: self.on_dxl(t, m), 10)
        rosout_qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Log, "/rosout", self.on_log, rosout_qos)
        for side in ("left", "right"):
            self.create_subscription(BatteryState, f"/ai_worker/battery/{side}/state",
                                     lambda m, s=side: self.on_batt(s, m), 10)
        self.create_timer(0.1, self.on_tick)
        self.create_timer(a.status_s, self.on_status)
        self.event("START", "watchdog",
                   "joints " + ", ".join(f"{j} limit={l:g}" for j, l in self.joints.items())
                   + f" csv={os.path.abspath(a.csv)}")

    # ------------------------------------------------------------------ util
    def event(self, kind, source, detail):
        t = time.monotonic() - self.t0
        wall = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"[{wall}] {kind:<12} {source:<28} {detail}", flush=True)
        self.csvw.writerow([wall, f"{t:.3f}", kind, source, detail])
        self.csvf.flush()

    # ---------------------------------------------------------- joint_states
    def on_js(self, m):
        now = time.monotonic()
        if self.stalled:
            self.event("JS_RESUME", "/joint_states", f"after {now - self.last_js:.2f} s")
            self.stalled = False
        self.last_js = now
        for j, lim in self.joints.items():
            if j not in m.name:
                continue
            i = m.name.index(j)
            pos = m.position[i] if i < len(m.position) else float("nan")
            cur = m.effort[i] if i < len(m.effort) else float("nan")
            s = self.st[j]
            s["n"] += 1
            s["pmin"], s["pmax"] = min(s["pmin"], pos), max(s["pmax"], pos)
            s["cmin"], s["cmax"] = min(s["cmin"], cur), max(s["cmax"], cur)
            ac = abs(cur)
            amp = f"{cur:+.0f} raw ({ac * MA_PER_RAW / 1000:.2f} A) pos {pos:.3f}"
            if lim > 0:
                frac = ac / lim
                if not s["near"] and frac >= self.a.warn_frac:
                    s["near"] = True
                    self.event("CUR_NEAR", j, f"{amp} = {frac:.0%} of limit {lim:g}")
                elif s["near"] and frac < self.a.warn_frac - 0.05:  # 5% hysteresis
                    s["near"] = False
                    self.event("CUR_OK", j, f"{amp} = {frac:.0%}")
                if frac >= self.a.sat_frac:
                    s["sat_since"] = s["sat_since"] or now
                    if not s["sat"] and now - s["sat_since"] >= self.a.sat_hold:
                        s["sat"] = True
                        self.event("CUR_SATURATED", j,
                                   f"{amp} >= {self.a.sat_frac:.0%} of limit for "
                                   f"{now - s['sat_since']:.1f} s")
                else:
                    if s["sat"]:
                        self.event("CUR_UNSAT", j, f"{amp} after "
                                   f"{now - s['sat_since']:.1f} s saturated")
                    s["sat_since"], s["sat"] = None, False
            # torque-loss heuristic: was loaded, now ~0 for a while
            if ac >= self.a.loaded_raw:
                s["loaded_since"] = now
                if s["lost"]:
                    self.event("CUR_BACK", j, amp)
                    s["lost"] = False
            elif (ac <= self.a.zero_raw and s["loaded_since"] is not None and not s["lost"]
                  and now - s["loaded_since"] >= self.a.zero_hold):
                s["lost"] = True
                self.event("CUR_COLLAPSE", j,
                           f"{amp}: was >= {self.a.loaded_raw:g} raw, ~0 for "
                           f"{self.a.zero_hold:g} s (torque off / released?)")

    def on_tick(self):
        if self.last_js is None or self.stalled:
            return
        gap = time.monotonic() - self.last_js
        if gap > self.a.stall_s:
            self.stalled = True
            self.event("JS_STALL", "/joint_states", f"no message for {gap:.2f} s")

    # ------------------------------------------------------------ dxl_state
    def on_dxl(self, topic, m):
        src = topic.split("/")[1]
        if self.comm.get(topic) != m.comm_state:
            if topic in self.comm or m.comm_state != 0:
                self.event("COMM", src, f"comm_state {self.comm.get(topic)} -> {m.comm_state}")
            self.comm[topic] = m.comm_state
        for i, id_ in enumerate(m.id):
            hw = m.dxl_hw_state[i] if i < len(m.dxl_hw_state) else 0
            key = (topic, id_)
            prev = self.hw.get(key)
            if prev != hw and (prev is not None or hw != 0):
                self.event("HW_ERROR" if hw else "HW_CLEAR", f"{src} ID {id_}",
                           f"0x{hw:02x} {decode_hw(hw)}" + (f" (was {decode_hw(prev)})"
                                                            if prev is not None else ""))
            self.hw[key] = hw

    # --------------------------------------------------------------- rosout
    def on_log(self, m):
        if m.level < Log.WARN or not any(n in m.name for n in ROSOUT_NODES):
            return
        kind = re.sub(r"[-+]?\d+(\.\d+)?", "#", m.msg)[:90]
        rec = self.rosout.get(kind)
        now = time.monotonic()
        if rec is None or now - rec[2] > self.a.log_quiet_s:
            if rec and rec[0] > 1:
                self.event("LOG_BURST_END", m.name, f"x{rec[0]} {kind}")
            lvl = {30: "WARN", 40: "ERROR", 50: "FATAL"}.get(m.level, str(m.level))
            self.event(f"LOG_{lvl}", m.name, m.msg[:200])
            self.rosout[kind] = [1, now, now]
        else:
            rec[0] += 1
            rec[2] = now

    # -------------------------------------------------------------- battery
    def on_batt(self, side, m):
        low = m.voltage < self.a.batt_min_v
        if low != self.batt_low.get(side, False):
            self.event("BATT_LOW" if low else "BATT_OK", f"battery {side}",
                       f"{m.voltage:.2f} V ({m.percentage:.0%})")
        self.batt_low[side] = low

    # --------------------------------------------------------------- status
    def on_status(self):
        parts = []
        for j, s in self.st.items():
            if s["n"]:
                parts.append(f"{j} pos {s['pmin']:.3f}..{s['pmax']:.3f} "
                             f"cur {s['cmin']:+.0f}..{s['cmax']:+.0f}")
            s.update(pmin=math.inf, pmax=-math.inf, cmin=math.inf, cmax=-math.inf, n=0)
        for kind, rec in self.rosout.items():
            if rec[0] > 1 and time.monotonic() - rec[2] > self.a.log_quiet_s:
                self.event("LOG_BURST_END", "rosout", f"x{rec[0]} {kind}")
                rec[0] = 1
        wall = datetime.now().strftime("%H:%M:%S")
        print(f"[{wall}] status       " + (" | ".join(parts) or "no joint_states yet"),
              flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--joint", action="append",
                    help="name[:current_limit_raw], repeatable (default "
                         "gripper_r_joint1:750, the XH540 Current Limit)")
    ap.add_argument("--warn-frac", type=float, default=0.8)
    ap.add_argument("--sat-frac", type=float, default=0.95)
    ap.add_argument("--sat-hold", type=float, default=1.0)
    ap.add_argument("--loaded-raw", type=float, default=50.0)
    ap.add_argument("--zero-raw", type=float, default=8.0)
    ap.add_argument("--zero-hold", type=float, default=0.5)
    ap.add_argument("--stall-s", type=float, default=0.3)
    ap.add_argument("--log-quiet-s", type=float, default=2.0,
                    help="a repeated log kind is collapsed until quiet this long")
    ap.add_argument("--batt-min-v", type=float, default=24.0)
    ap.add_argument("--status-s", type=float, default=5.0)
    ap.add_argument("--csv", default=f"/tmp/motor_watchdog_{datetime.now():%Y%m%d_%H%M%S}.csv")
    a = ap.parse_args()
    a.joint = a.joint or ["gripper_r_joint1:750"]
    rclpy.init()
    n = Watchdog(a)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.event("STOP", "watchdog", "")
        n.csvf.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
