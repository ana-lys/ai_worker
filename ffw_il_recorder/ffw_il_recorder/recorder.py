#!/usr/bin/env python3
"""30 Hz imitation-learning (IL) episode recorder.

Decodes the two UDP RTP/MJPEG feeds an operator needs to record a demo - the
OAK-D 720p head view (port 9110) and the right-wrist D405 RGB view (cam1 "IR"
port, 9003, rotated CCW after decode) - straight off the network, with decode
pipelines copied verbatim from ``ffw_stream/src/realsense_udp_receiver.cpp``.
Video is never on ROS topics here; the receiver and this recorder cannot share
the unicast ports, so a recording session replaces the receiver launch.

State capture runs on a 30 Hz sampler thread: on every tick it snapshots the
latest /joint_states and the latest *achieved* EE poses, decodes any new video
frame into a per-stream JPEG, and appends one uniform state.csv row. Frames are
written only when a new decode actually arrived; every row references the frame
index (and receive time) that was current at that tick, so the dataloader
nearest-samples without forward-filling.

Episode control is keyboard-only:

    SPACE  toggle: first tap starts an episode (creates episode_NNNN/ +
           state.csv + meta.json), next tap stops + finalizes
    R      abort the current episode: stop and DELETE the partial
           episode_XXXX/ dir (no meta.json -> not recorded, not loadable)
    Q      quit recorder          (Ctrl-C quits too)

An episode layout (episode_0000/, episode_0001/, ...):

    state.csv                  one row per 30 Hz tick (uniform)
    head/seq_000000.jpg ...    every decoded head frame
    right_rgb/seq_000000.jpg   every decoded right-wrist frame
    meta.json                  params, timings, counts, joint names, oakd K/D

state.csv columns: ``t`` (seconds, 0 at first row), ``q_<joint>`` per joint of
the start JointState, ``ee_l_*`` / ``ee_r_*`` (achieved pose xyz + quat),
``gripper_l_delta`` / ``gripper_r_delta`` (per-arrival delta of the gripper
joints), then ``head_frame``, ``head_recv_t``, ``right_frame``,
``right_recv_t`` (index + receive time of the frame current at that tick).

    ros2 launch ffw_il_recorder il_recorder_launch.py
"""

import csv
import json
import logging
import os
import queue
import re
import select
import shutil
import sys
import termios
import threading
import time
import tty

import cv2
import numpy as np

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, CameraInfo

from ffw_il_recorder import gst_decode

log = logging.getLogger("ffw_il_recorder.recorder")

JOINT_TOPIC = "/joint_states"
EE_TOPIC_L = "/ik_solver/achieved_ee_pose_l"
EE_TOPIC_R = "/ik_solver/achieved_ee_pose_r"
CAM_INFO_TOPIC = "/oakd/camera_info"

GRIPPER_L_PREFIX = "gripper_l"
GRIPPER_R_PREFIX = "gripper_r"

DEFAULT_RECORD_DIR = os.path.expanduser("~/robotis_ws/records")


def r6(v):
    """Round a float to 6 decimals (state.csv convention)."""
    return round(float(v), 6)


class _EpisodeState:
    """Tiny shared status struct (sampler writes, display thread reads)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.status = "idle"        # 'idle' | 'rec'
        self.episode = None         # int index, set while recording
        self.rows = 0
        self.head = 0               # JPEGs written
        self.right = 0
        self.error = ""             # last refusal reason (start blocked)
        self.recorded = 0           # completed episodes (on disk + finalized)
        self.ep_start = 0.0         # time.time() when the live episode began
        self.last_idx = None        # most recent completed episode index
        self.last_rows = 0          # ... and its step count / duration
        self.last_dur = 0.0


class ILRecorderNode(Node):
    """Caches newest msgs, runs the decoders + sampler + input threads."""

    def __init__(self):
        super().__init__("il_episode_recorder")

        self.declare_parameter("rgb_source", "oakd_lite")
        self.declare_parameter("record_dir", DEFAULT_RECORD_DIR)
        self.declare_parameter("sample_hz", 30.0)
        self.declare_parameter("jpeg_quality", 95)
        self.declare_parameter("oakd_codec", "h264")
        self.declare_parameter("rs_codec", "h264")
        self.declare_parameter("oakd_720p_video_port", 9110)
        self.declare_parameter("right_rgb_port", 9003)
        self.declare_parameter("headless", False)

        self.record_dir = str(self.get_parameter("record_dir").value)
        self.sample_hz = float(self.get_parameter("sample_hz").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.oakd_codec = str(self.get_parameter("oakd_codec").value)
        self.rs_codec = str(self.get_parameter("rs_codec").value)
        self.oakd_port = int(self.get_parameter("oakd_720p_video_port").value)
        self.right_port = int(self.get_parameter("right_rgb_port").value)
        self.headless = bool(self.get_parameter("headless").value)

        # ---- newest-message cache (subscription callbacks write, sampler reads)
        self._lock = threading.Lock()
        self._joints = None         # sensor_msgs/JointState
        self._ee_l = None           # geometry_msgs/PoseStamped
        self._ee_r = None
        self._cam_info = None       # sensor_msgs/CameraInfo
        self._prev_gripper = {GRIPPER_L_PREFIX: None, GRIPPER_R_PREFIX: None}
        self._gripper_delta = {GRIPPER_L_PREFIX: 0.0, GRIPPER_R_PREFIX: 0.0}

        qos_live = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.VOLATILE,
                              depth=1)
        qos_info = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              depth=1)
        self.create_subscription(JointState, JOINT_TOPIC,
                                 self._on_joints, qos_live)
        self.create_subscription(PoseStamped, EE_TOPIC_L,
                                 self._on_ee_l, qos_live)
        self.create_subscription(PoseStamped, EE_TOPIC_R,
                                 self._on_ee_r, qos_live)
        self.create_subscription(CameraInfo, CAM_INFO_TOPIC,
                                 self._on_cam_info, qos_info)

        # ---- control plane
        self._done = threading.Event()
        self._cmds = queue.Queue()
        self._last_toggle = 0.0        # monotonic stamp for the SPACE debounce
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._st = _EpisodeState()
        self._st.recorded = self._count_recorded_episodes()

        # ---- episode writer state (sampler thread only)
        self._ep = None             # open episode context dict, or None
        self._threads = []

    # ------------------------------------------------------------- callbacks
    def _on_joints(self, msg):
        names, pos = list(msg.name), list(msg.position)
        with self._lock:
            self._joints = msg
            for prefix in (GRIPPER_L_PREFIX, GRIPPER_R_PREFIX):
                i = next((j for j, n in enumerate(names) if n.startswith(prefix)),
                         -1)
                if i < 0:
                    continue
                cur = pos[i]
                prev = self._prev_gripper[prefix]
                if prev is not None:
                    self._gripper_delta[prefix] = float(cur) - float(prev)
                self._prev_gripper[prefix] = float(cur)

    def _on_ee_l(self, msg):
        with self._lock:
            self._ee_l = msg

    def _on_ee_r(self, msg):
        with self._lock:
            self._ee_r = msg

    def _on_cam_info(self, msg):
        with self._lock:
            self._cam_info = msg

    # ----------------------------------------------------------- snapshots
    def _joint_msg(self):
        with self._lock:
            return self._joints

    def _state_cells(self):
        """(q cells for header joint names, 7+7 EE cells, gripper deltas)."""
        with self._lock:
            msg, l, r = self._joints, self._ee_l, self._ee_r
            gl, gr = (self._gripper_delta[GRIPPER_L_PREFIX],
                      self._gripper_delta[GRIPPER_R_PREFIX])
        q = {}
        if msg is not None:
            for n, v in zip(msg.name, msg.position):
                q[n] = float(v)
        q_cells = [r6(q[n]) if n in q else "" for n in self._header_joint_names]
        ee_cells = []
        for m in (l, r):
            if m is None:
                ee_cells += [""] * 7
            else:
                p, o = m.pose.position, m.pose.orientation
                ee_cells += [r6(p.x), r6(p.y), r6(p.z),
                             r6(o.x), r6(o.y), r6(o.z), r6(o.w)]
        return q_cells + ee_cells + [r6(gl), r6(gr)]

    # ----------------------------------------------------------- run / stop
    def run(self):
        log.info(
            "record_dir=%s  sample_hz=%.1f  jpeg_quality=%d",
            self.record_dir, self.sample_hz, self.jpeg_quality)
        log.info(
            "head : udp port %d (%s)   right_rgb: udp port %d (%s)",
            self.oakd_port, self.oakd_codec, self.right_port, self.rs_codec)

        os.makedirs(self.record_dir, exist_ok=True)

        # Video decoders (verbatim receiver pipelines).
        self._head = gst_decode.VideoDecoder(
            "head", self.oakd_port, codec=self.oakd_codec, feed="oakd720p")
        self._right = gst_decode.VideoDecoder(
            "right_rgb", self.right_port, codec=self.rs_codec, feed="rs",
            rotate=cv2.ROTATE_90_COUNTERCLOCKWISE)
        self._streams = {
            "head": {"dec": self._head, "subdir": "head",
                     "last_idx": None, "last_recv": None},
            "right": {"dec": self._right, "subdir": "right_rgb",
                      "last_idx": None, "last_recv": None},
        }
        self._head.start()
        self._right.start()

        # ROS spin in a background thread. Kept out of self._threads because
        # it only exits after executor.shutdown() (done in _shutdown, where it
        # is joined separately).
        spin = threading.Thread(target=self._executor.spin, name="rclpy-spin",
                                daemon=True)
        self._spin_thread = spin
        spin.start()

        # Sampler: the only thread that touches episode files.
        sampler = threading.Thread(target=self._sampler_loop,
                                   name="sampler", daemon=True)
        sampler.start()
        self._threads.append(sampler)

        # Input paths: window keys (waitKey) if a display is up, raw-stdin keys
        # when this process owns a terminal, so focus can live on either.
        use_window = (not self.headless
                      and (os.environ.get("DISPLAY")
                           or os.environ.get("WAYLAND_DISPLAY")))
        use_stdin = sys.stdin.isatty()
        if use_window:
            dth = threading.Thread(target=self._display_loop, name="display",
                                   daemon=True)
            dth.start()
            self._threads.append(dth)
        if use_stdin:
            kth = threading.Thread(target=self._keyboard_loop, name="keys",
                                   daemon=True)
            kth.start()
            self._threads.append(kth)
        if not use_window and not use_stdin:
            log.warning(
                "no display and no terminal stdin - keyboard control "
                "unavailable; run under `ros2 launch` from a terminal, or "
                "set headless:=true with a terminal open")

        self._print_help()
        try:
            while not self._done.wait(0.25):
                pass
        except KeyboardInterrupt:
            log.info("Ctrl-C received, shutting down")
        self._shutdown()
        return 0

    def _print_help(self):
        log.info(
            "SPACE = start/stop episode   R = abort/discard   Q = quit\n"
            "  (tap in this terminal, or in the video window)")

    def _shutdown(self):
        self._done.set()
        # Join input + sampler threads first. The sampler owns any open episode
        # and its finally-block finalizes it on its own thread, so no writer
        # races on state.csv during teardown. Keyboard/display just exit on
        # _done (they restore the terminal / destroy the window in finally).
        for t in self._threads:
            try:
                t.join(timeout=5.0)
            except Exception:                          # noqa: BLE001
                pass
        # Belt-and-braces: if the sampler died without finalizing (crash before
        # its finally, or a wedged join), close the episode here - nothing else
        # touches episode state now that the sampler is joined.
        if self._st.status == "rec":
            try:
                self._finalize_episode()
            except Exception:                          # noqa: BLE001
                log.error("finalize during shutdown failed",
                                        exc_info=True)
        # Stop the decode threads, then unblock the rclpy spin thread (it only
        # returns after executor.shutdown()).
        try:
            self._head.stop()
            self._right.stop()
        except Exception:                              # noqa: BLE001
            log.error("decoder stop failed", exc_info=True)
        self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=3.0)
        self.destroy_node()
        # Context is still up: rclpy handlers were uninstalled in main(),
        # so nothing shuts the context down behind our back before this.
        if rclpy.ok():
            rclpy.shutdown()
        log.info("bye")

    # ------------------------------------------------------------- input
    # One tap of SPACE starts the episode, the next tap stops it. Holding
    # space down would otherwise fire the OS key-repeat stream; the debounce
    # swallows repeats so a hold can't flip record on/off several times/sec.
    TOGGLE_DEBOUNCE_S = 0.35

    def _on_char(self, c):
        c = c.lower()
        if c == " ":
            now = time.monotonic()
            if now - self._last_toggle >= self.TOGGLE_DEBOUNCE_S:
                self._last_toggle = now
                self._cmds.put("toggle")
        elif c == "r":
            self._cmds.put("abort")
        elif c in ("q", "\x03"):
            self._cmds.put("quit")

    def _keyboard_loop(self):
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while not self._done.is_set():
                rd, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not rd:
                    continue
                data = os.read(fd, 64)
                if not data:
                    continue
                for b in data:
                    self._on_char(chr(b))
        except Exception:                              # noqa: BLE001
            log.error("keyboard loop failed", exc_info=True)
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            except Exception:                          # noqa: BLE001
                pass

    def _display_loop(self):
        win = "ffw_il_recorder  |  head  |  right wrist RGB"
        try:
            cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)
        except Exception as e:                         # noqa: BLE001
            log.error("cannot open display window: %s", e)
            return
        last_render = 0.0
        try:
            while not self._done.is_set():
                key = cv2.waitKey(15) & 0xFF
                if key not in (0xFF, 255, -1):
                    try:
                        self._on_char(chr(key))
                    except ValueError:
                        pass
                now = time.monotonic()
                if now - last_render >= 0.05:
                    last_render = now
                    view = self._compose_view()
                    if view is not None:
                        cv2.imshow(win, view)
        except Exception:                              # noqa: BLE001
            log.error("display loop failed", exc_info=True)
        finally:
            try:
                cv2.destroyWindow(win)
            except Exception:                          # noqa: BLE001
                pass

    def _compose_view(self):
        """Side-by-side live view with a status banner (display thread)."""
        head, _ = self._head.peek()
        right, _ = self._right.peek()

        def fit(img, dh, label):
            if img is None or img.size == 0:
                blank = np.zeros((dh, int(dh * 16 / 9), 3), np.uint8)
                cv2.putText(blank, label + " (no video)", (12, dh - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1,
                            cv2.LINE_AA)
                return blank
            h, w = img.shape[:2]
            return cv2.resize(img, (int(w * dh / h), dh),
                              interpolation=cv2.INTER_AREA)

        pane_h = 540
        a = fit(head, pane_h, "head")
        b = fit(right, pane_h, "right_wrist")
        view = cv2.hconcat([a, b])
        if view.shape[1] > 1900:
            view = cv2.resize(view, (1900, int(view.shape[0] * 1900 / view.shape[1])))

        with self._st.lock:
            status, rows, nhead, nright, err = (
                self._st.status, self._st.rows,
                self._st.head, self._st.right, self._st.error)
            recorded = self._st.recorded
            episode = self._st.episode
            ep_start = self._st.ep_start
            last_idx = self._st.last_idx
            last_rows = self._st.last_rows
            last_dur = self._st.last_dur
        if status == "rec":
            banner = ("REC  rows=%d  head=%d  right=%d   (space stops, r aborts)" %
                      (rows, nhead, nright))
        elif err:
            banner = "IDLE  " + err
        else:
            banner = "IDLE   space=start/stop  Q=quit"
        cv2.rectangle(view, (0, 0), (view.shape[1], 52), (0, 0, 0), -1)
        cv2.putText(view, banner, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 0, 255) if status == "rec" else (60, 255, 60), 2,
                    cv2.LINE_AA)
        cv2.putText(view,
                    "head %.1f fps | right %.1f fps" %
                    (self._head.fps, self._right.fps),
                    (12, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (200, 200, 200), 1, cv2.LINE_AA)

        # Bottom band: how many episodes have been recorded, plus live/elapsed
        # info. 'step' = 30 Hz ticks written = state.csv rows so far.
        H = view.shape[0]
        if status == "rec":
            foot = "episodes recorded: %d   ep_%04d  step %d  %.1fs" % (
                recorded, episode, rows, time.time() - ep_start)
        elif last_idx is not None:
            foot = "episodes recorded: %d   last ep_%04d: %d steps, %.1fs" % (
                recorded, last_idx, last_rows, last_dur)
        else:
            foot = "episodes recorded: %d" % recorded
        cv2.rectangle(view, (0, H - 40), (view.shape[1], H), (0, 0, 0), -1)
        cv2.putText(view, foot, (12, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (230, 230, 230), 1, cv2.LINE_AA)
        return view

    # ------------------------------------------------------------- commands
    def _drain_commands(self):
        while True:
            try:
                cmd = self._cmds.get_nowait()
            except queue.Empty:
                break
            if cmd == "toggle":
                if self._st.status == "rec":
                    self._cmd_stop()
                else:
                    self._cmd_start()
            elif cmd == "abort":
                self._cmd_abort()
            elif cmd == "quit":
                log.info("Quit requested")
                self._done.set()

    def _cmd_start(self):
        if self._st.status == "rec":
            return
        joint_msg = self._joint_msg()
        if joint_msg is None or not joint_msg.name:
            log.error(
                "Cannot start: no /joint_states received yet.")
            with self._st.lock:
                self._st.error = "no joint_states yet"
            return

        idx = self._next_episode_index()
        ep_dir = os.path.join(self.record_dir, "episode_%04d" % idx)
        os.makedirs(os.path.join(ep_dir, "head"), exist_ok=True)
        os.makedirs(os.path.join(ep_dir, "right_rgb"), exist_ok=True)

        self._header_joint_names = list(joint_msg.name)
        header = self._csv_header(self._header_joint_names)
        fh = open(os.path.join(ep_dir, "state.csv"), "w", newline="")
        csvw = csv.writer(fh)
        csvw.writerow(header)

        self._ep = {
            "dir": ep_dir, "idx": idx, "fh": fh, "w": csvw,
            "rows": 0, "t0": None, "closed": False,
            "start_wall": time.time(),
            "start_wall_iso": time.strftime("%Y-%m-%d %H:%M:%S"),
            "counts": {"head": 0, "right": 0},
        }
        for s in self._streams.values():
            s["last_idx"] = None
            s["last_recv"] = None

        with self._st.lock:
            self._st.status = "rec"
            self._st.episode = idx
            self._st.rows = 0
            self._st.head = 0
            self._st.right = 0
            self._st.error = ""
            self._st.ep_start = time.time()
        log.info("==> episode %s started (rows will be ~%.0f "
                               "per second) <==", ep_dir, self.sample_hz)

    def _csv_header(self, joint_names):
        cols = ["t"]
        cols += ["q_%s" % n for n in joint_names]
        cols += ["ee_l_x", "ee_l_y", "ee_l_z",
                 "ee_l_qx", "ee_l_qy", "ee_l_qz", "ee_l_qw"]
        cols += ["ee_r_x", "ee_r_y", "ee_r_z",
                 "ee_r_qx", "ee_r_qy", "ee_r_qz", "ee_r_qw"]
        cols += ["gripper_l_delta", "gripper_r_delta"]
        cols += ["head_frame", "head_recv_t", "right_frame", "right_recv_t"]
        return cols

    def _count_recorded_episodes(self):
        """Completed episodes already on disk (have a meta.json)."""
        pat = re.compile(r"^episode_(\d{4,})$")
        n = 0
        try:
            for name in os.listdir(self.record_dir):
                m = pat.match(name)
                if m and os.path.exists(os.path.join(self.record_dir, name,
                                                     "meta.json")):
                    n += 1
        except OSError:
            pass
        return n

    def _next_episode_index(self):
        pat = re.compile(r"^episode_(\d{4,})$")
        best = -1
        try:
            for name in os.listdir(self.record_dir):
                m = pat.match(name)
                if m:
                    best = max(best, int(m.group(1)))
        except OSError:
            pass
        return best + 1

    def _cmd_stop(self):
        if self._st.status == "rec":
            self._finalize_episode()
        else:
            log.info("No episode in progress to stop.")

    def _cmd_abort(self):
        if self._st.status == "rec":
            self._abort_episode()
        else:
            log.info("No episode in progress to abort.")

    def _abort_episode(self):
        """Discard the in-progress take: close the CSV and DELETE the whole
        episode_XXXX/ dir. Aborted takes never get a meta.json, so they do not
        count toward 'episodes recorded' and the loader never sees them."""
        ep = self._ep
        if ep is None or ep.get("closed"):
            return
        ep_dir = ep["dir"]
        rows = ep["rows"]
        try:
            ep["fh"].close()
        finally:
            ep["closed"] = True
        shutil.rmtree(ep_dir, ignore_errors=True)
        with self._st.lock:
            self._st.status = "idle"
            self._st.episode = None
            self._st.rows = 0
            self._st.head = 0
            self._st.right = 0
            self._st.error = ""
            self._st.ep_start = 0.0
        log.info("==> episode %s ABORTED: %d rows discarded, dir removed <==",
                 ep_dir, rows)
        self._ep = None

    def _finalize_episode(self):
        ep = self._ep
        if ep is None or ep.get("closed"):
            return
        rows = ep["rows"]
        duration = time.time() - ep["start_wall"]
        # CameraInfo snapshot (transient-local, republished 1 Hz).
        with self._lock:
            ci = self._cam_info
        meta = {
            "episode": ep["idx"],
            "record_dir": self.record_dir,
            "start_wall": ep["start_wall_iso"],
            "end_wall": time.strftime("%Y-%m-%d %H:%M:%S"),
            "duration_s": round(duration, 3),
            "sample_hz": self.sample_hz,
            "jpeg_quality": self.jpeg_quality,
            "rows": rows,
            "joint_topic": JOINT_TOPIC,
            "ee_topics": {"left": EE_TOPIC_L, "right": EE_TOPIC_R},
            "joint_names": list(self._header_joint_names),
            "state_csv_columns": self._csv_header(self._header_joint_names),
            "streams": {},
            "camera_info": self._camera_info_dict(ci),
            "notes": ("right_rgb is D405 cam1 'IR' (UDP %d) decoded and rotated "
                      "90 deg CCW; EE columns are the *achieved* end-effector "
                      "poses." % self.right_port),
        }
        for key, s in self._streams.items():
            dec = s["dec"]
            meta["streams"][key] = {
                "port": dec.port,
                "codec": ("mjpeg" if "JPEG" in dec.pipeline else "h264"),
                "frames_written": ep["counts"][key],
                "width": dec.width,
                "height": dec.height,
                "fps": round(dec.fps, 1),
                "connected": dec.connected,
            }
        meta_path = os.path.join(ep["dir"], "meta.json")
        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)
        try:
            ep["fh"].flush()
        finally:
            # Mark closed in the same breath: a re-entrant finalize (shutdown
            # belt-and-braces) must never close an already-closed handle.
            ep["fh"].close()
            ep["closed"] = True

        with self._st.lock:
            self._st.status = "idle"
            self._st.episode = None
            self._st.rows = ep["rows"]
            self._st.recorded += 1
            self._st.last_idx = ep["idx"]
            self._st.last_rows = ep["rows"]
            self._st.last_dur = duration
            self._st.ep_start = 0.0
        log.info(
            "==> episode %s done: %d rows, head %d jpg, right %d jpg, "
            "%.1fs  (meta.json written) <==",
            ep["dir"], rows, ep["counts"]["head"], ep["counts"]["right"],
            duration)
        self._ep = None

    @staticmethod
    def _camera_info_dict(ci):
        if ci is None:
            return None
        return {
            "frame_id": ci.header.frame_id,
            "width": ci.width,
            "height": ci.height,
            "distortion_model": ci.distortion_model,
            "k": [float(v) for v in ci.k],
            "d": [float(v) for v in ci.d],
        }

    # ------------------------------------------------------------- sampler
    def _sampler_loop(self):
        period = 1.0 / self.sample_hz
        t0 = time.monotonic()
        last_tick = -1
        try:
            while not self._done.is_set():
                self._drain_commands()
                idx = int((time.monotonic() - t0) / period)
                if idx > last_tick:
                    last_tick = idx
                    self._tick()
                # Sleep exactly to the next tick boundary (never busy-spin);
                # if a tick overran we are already past it and only nap.
                due = t0 + (last_tick + 1) * period - time.monotonic()
                if due > 0.001:
                    self._done.wait(due)
                else:
                    self._done.wait(0.0005)
        except Exception:                              # noqa: BLE001
            log.error("sampler crashed", exc_info=True)
        finally:
            if self._st.status == "rec":
                try:
                    self._finalize_episode()
                except Exception:                      # noqa: BLE001
                    log.error(
                        "finalize after sampler crash failed", exc_info=True)
            self._done.set()

    def _tick(self):
        if self._st.status != "rec":
            return
        self._record_row()

    def _record_row(self):
        ep = self._ep
        now = time.time()
        if ep["t0"] is None:
            ep["t0"] = now
        t = now - ep["t0"]

        # Frames first: write any new decode, so the row can reference files
        # that exist. snapshot() reports new since the last tick (idle gaps do
        # not consume the flag; the sampler is the only snapshot() caller).
        frame_cells = []
        for key in ("head", "right"):
            s = self._streams[key]
            frame, recv_t, is_new = s["dec"].snapshot()
            if is_new and frame is not None and frame.size > 0:
                idx = ep["counts"][key]
                path = os.path.join(ep["dir"], s["subdir"],
                                    "seq_%06d.jpg" % idx)
                ok, buf = cv2.imencode(".jpg", frame,
                                       (cv2.IMWRITE_JPEG_QUALITY,
                                        self.jpeg_quality))
                if ok:
                    with open(path, "wb") as f:
                        f.write(buf.tobytes())
                    ep["counts"][key] = idx + 1
                    s["last_idx"] = idx
                    s["last_recv"] = recv_t
            if s["last_idx"] is None:
                frame_cells += ["", ""]
            else:
                frame_cells += [s["last_idx"], r6(s["last_recv"] - ep["t0"])]

        row = [r6(t)] + self._state_cells() + frame_cells
        ep["w"].writerow(row)
        ep["rows"] += 1
        ep["fh"].flush()

        with self._st.lock:
            self._st.rows = ep["rows"]
            self._st.head = ep["counts"]["head"]
            self._st.right = ep["counts"]["right"]


def main():
    logging.basicConfig(level=logging.INFO,
                        format="[%(name)s] %(levelname)s: %(message)s",
                        stream=sys.stderr)
    rclpy.init(args=sys.argv[1:])
    # rclpy installs C-level SIGINT/SIGTERM handlers that shut the context
    # down asynchronously on Ctrl-C (probe-verified). That races this node's
    # own KeyboardInterrupt teardown and turns the spin thread into an
    # ExternalShutdownException, so uninstall them: recorder owns the signal.
    rclpy.signals.uninstall_signal_handlers()
    node = ILRecorderNode()
    return node.run()


if __name__ == "__main__":
    sys.exit(main())
