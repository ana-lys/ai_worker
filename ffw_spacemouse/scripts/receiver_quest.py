"""Controller telemetry monitor for Hand Tracking Streamer (HTS).

Listens for controller packets and decodes them into readable state and events:
    - ``Right controller:`` state lines are printed as a live one-line summary.
    - ``Right controller event:`` lines are printed as they arrive.
    - ``--debug``: focused trigger/grip/thumbrest readout plus EDGE/CHECK lines.
      Verifies grip/trigger are real analog signals and whether the thumbrest
      touch bit ever fires — the quest teleop plan's keep-alive depends on this
      (see §10 decision 7 / §13 #11).

Hand, head and any other telemetry lines are ignored.

Each controller state CSV has 13 comma-separated fields::

    0-2    position (px, py, pz)
    3-6    orientation quaternion (qx, qy, qz, qw)     ← new in this version
    7      trigger (float 0-1)
    8      grip (float 0-1)
    9-10   thumbstick (x, y)
    11     buttons bitmask
    12     touches bitmask

Examples:
    python controller_monitor.py --protocol udp --host 0.0.0.0 --port 9000
    python controller_monitor.py --protocol tcp --host localhost --port 8000
    python controller_monitor.py --debug   # verify grip/trigger signals + whether thumbrest ever fires
"""

from __future__ import annotations

import argparse
import socket
import sys
import threading
import time
from typing import Iterator, Optional

# Bit layout shared with ControllerStreamer.cs / CONNECTIONS.md.
BUTTON_BITS = (
    (0, "trigger"),
    (1, "grip"),
    (2, "A/X"),
    (3, "B/Y"),
    (4, "thumbstick"),
    (5, "menu"),
)

TOUCH_BITS = (
    (0, "trigger"),
    (1, "thumbrest"),
    (2, "A/X"),
    (3, "B/Y"),
    (4, "thumbstick"),
)


def decode_mask(mask: int, bits) -> str:
    """Render a bitmask as a space separated list of active names."""
    names = [name for bit, name in bits if mask & (1 << bit)]
    return " ".join(names) if names else "-"


def strip_debug_header(label: str) -> str:
    """Drop the ``| f = N | t = NS`` metadata the debug toggle adds to labels."""
    return label.split("|", 1)[0].strip()


def format_state(label: str, values: list[str]) -> Optional[str]:
    """Format a controller state line, or None if the payload is malformed.

    Expected CSV layout (13 fields shared with ControllerStreamer.cs):

        Index   Field
        ------  -------------------------
        0-2     position (px, py, pz)
        3-6     orientation quaternion (qx, qy, qz, qw)
        7-8     trigger, grip  (float 0-1)
        9-10    thumbstick (x, y)
        11      buttons bitmask
        12      touches bitmask
    """
    if len(values) < 13:
        return None

    try:
        px, py, pz = (float(v) for v in values[0:3])
        qx, qy, qz, qw = (float(v) for v in values[3:7])
        trigger, grip = float(values[7]), float(values[8])
        stick_x, stick_y = float(values[9]), float(values[10])
        buttons, touches = int(float(values[11])), int(float(values[12]))
    except ValueError:
        return None

    return (
        f"{label:<18} "
        f"pos ({px:+.3f}, {py:+.3f}, {pz:+.3f})  "
        f"quat ({qx:+.3f}, {qy:+.3f}, {qz:+.3f}, {qw:+.3f})  "
        f"trig {trigger:.2f}  grip {grip:.2f}  "
        f"stick ({stick_x:+.2f}, {stick_y:+.2f})  "
        f"btn [{decode_mask(buttons, BUTTON_BITS)}]  "
        f"touch [{decode_mask(touches, TOUCH_BITS)}]"
    )


STATE_PRINT_INTERVAL_S = 0.2
_last_state_print: dict[str, float] = {}

# --- debug mode (--debug) ---------------------------------------------------
# Focused trigger/grip/thumbrest readout. The quest teleop plan's keep-alive is
# "both thumbs ≥ 0.5" where "thumb" is the analog GRIP (plan §10 decision 7).
# This mode verifies (a) grip/trigger are real independent analog signals and
# (b) whether the thumbrest-touch bit (TOUCH_BITS bit 1) ever fires at all —
# hardware showed it never does, so it cannot back the keep-alive (§13 #11).
DEBUG_PRINT_INTERVAL_S = 0.1          # 10 Hz live readout in debug mode
_DEBUG = False
_last_grip_thumb: dict[str, tuple[int, int]] = {}  # label -> (grip_bit, thumb_bit)
_edge_together: dict[str, int] = {}                # label -> edges where BOTH moved
_edge_apart: dict[str, int] = {}                   # label -> edges where only ONE moved
_thumb_ever_seen: dict[str, bool] = {}             # label -> thumb bit EVER 1
_edge_report_time = 0.0


def _bit_label(bit: int) -> str:
    return "on" if bit else "off"


def format_debug_state(label: str, values: list[str]) -> Optional[str]:
    """Compact live readout of the signals the teleop plan depends on.

    Shows, per controller: trigger and grip (analog float + button bit), the
    thumbrest and thumbstick touch bits, and the stick axes.
    """
    if len(values) < 13:
        return None
    try:
        trigger, grip = float(values[7]), float(values[8])
        stick_x, stick_y = float(values[9]), float(values[10])
        buttons, touches = int(float(values[11])), int(float(values[12]))
    except ValueError:
        return None

    thumb = []
    if touches & (1 << 1):
        thumb.append("rest")
    if touches & (1 << 4):
        thumb.append("stick")
    return (
        f"{label:<16} trig {trigger:.2f}[{_bit_label(buttons & 1)}]  "
        f"grip {grip:.2f}[{_bit_label(buttons & (1 << 1))}]  "
        f"thumb[{','.join(thumb) or '-'}]  stick({stick_x:+.2f},{stick_y:+.2f})"
    )


def _debug_check_edge(label: str, values: list[str]) -> bool:
    """Watch grip vs thumbrest-touch for coupling (--debug only).

    Returns True when an edge fired this frame (an EDGE line was printed, so the
    throttled state line for that frame is suppressed).

    Verdict logic: an APART edge (exactly one of grip/thumbrest changed) proves
    they come from separate sensors. If every edge is SAME all session, the
    thumbrest-touch is slaved to grip and the plan's keep-alive would need the
    analog grip instead (see plan §10 decision 5 / finding #11).
    """
    if len(values) < 13:
        return False
    try:
        trigger, grip = float(values[7]), float(values[8])
        buttons, touches = int(float(values[11])), int(float(values[12]))
    except ValueError:
        return False

    grip_bit = buttons & (1 << 1)     # grip button
    thumb_bit = touches & (1 << 1)    # thumbrest touch

    _thumb_ever_seen[label] = _thumb_ever_seen.get(label, False) or bool(thumb_bit)

    last = _last_grip_thumb.get(label)
    _last_grip_thumb[label] = (grip_bit, thumb_bit)
    if last is None:
        return False   # first frame — nothing to compare yet

    prev_grip, prev_thumb = last
    grip_moved = grip_bit != prev_grip
    thumb_moved = thumb_bit != prev_thumb
    if not grip_moved and not thumb_moved:
        return False

    if grip_moved and thumb_moved:
        _edge_together[label] = _edge_together.get(label, 0) + 1
        verdict = "SAME"
    else:
        _edge_apart[label] = _edge_apart.get(label, 0) + 1
        verdict = "APART"
    print(
        f"EDGE {label:<16} grip {_bit_label(prev_grip)}->{_bit_label(grip_bit)}  "
        f"thumb {_bit_label(prev_thumb)}->{_bit_label(thumb_bit)}  [{verdict}]  "
        f"(grip={grip:.2f} trig={trigger:.2f})",
        flush=True)

    global _edge_report_time
    now = time.monotonic()
    if now - _edge_report_time >= 1.0:
        _edge_report_time = now
        for lbl in _last_grip_thumb:
            together = _edge_together.get(lbl, 0)
            apart = _edge_apart.get(lbl, 0)
            if not _thumb_ever_seen.get(lbl, False):
                verdict = ("thumbrest NEVER fired (touches mask stays 0) -> "
                           "cannot back keep-alive; use analog grip")
            elif apart == 0:
                verdict = ("thumbrest SLAVED to grip -> likely ONE sensor; "
                           "use analog grip for keep-alive")
            else:
                verdict = ("thumbrest fires independently of grip -> SEPARATE "
                           "sensors; either can back the keep-alive")
            print(f"CHECK {lbl:<16} same-edge={together} apart-edge={apart}  {verdict}",
                  flush=True)
    return True


def handle_line(line: str) -> None:
    """Decode and print one telemetry line if it is controller data.

    State packets arrive at up to 100 Hz per controller, so they are throttled to a
    readable rate. Events are never throttled - every edge is printed.
    """
    if ":" not in line:
        return

    raw_label, _, payload = line.partition(":")
    label = strip_debug_header(raw_label)
    if "controller" not in label:
        return

    values = [v.strip() for v in payload.split(",") if v.strip()]

    if label.endswith("controller event"):
        if len(values) >= 2:
            print(f"EVENT  {label:<24} {values[0]} {values[1]}", flush=True)
        return

    if _DEBUG:
        if _debug_check_edge(label, values):
            return  # edge + CHECK lines already printed for this frame
        now = time.monotonic()
        if now - _last_state_print.get(label, 0.0) < DEBUG_PRINT_INTERVAL_S:
            return
        state = format_debug_state(label, values)
        if state:
            _last_state_print[label] = now
            print(state, flush=True)
        return

    now = time.monotonic()
    if now - _last_state_print.get(label, 0.0) < STATE_PRINT_INTERVAL_S:
        return

    state = format_state(label, values)
    if state:
        _last_state_print[label] = now
        print(state, flush=True)


def iter_udp_lines(host: str, port: int) -> Iterator[str]:
    """Yield decoded lines from a UDP socket."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    print(f"Listening for UDP on {host}:{port}", file=sys.stderr)

    try:
        while True:
            data, _ = sock.recvfrom(65536)
            for line in data.decode("utf-8", errors="replace").split("\n"):
                if line:
                    yield line
    finally:
        sock.close()


def serve_tcp(host: str, port: int) -> None:
    """Accept TCP connections (one per streamer component) and decode each stream."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(8)
    print(f"Listening for TCP on {host}:{port}", file=sys.stderr)

    def handle_conn(conn: socket.socket, addr) -> None:
        print(f"Accepted connection from {addr}", file=sys.stderr)
        buffer = ""
        with conn:
            while True:
                data = conn.recv(4096)
                if not data:
                    break
                buffer += data.decode("utf-8", errors="replace")
                *lines, buffer = buffer.split("\n")
                for line in lines:
                    if line:
                        handle_line(line)
        print(f"Connection from {addr} closed", file=sys.stderr)

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(target=handle_conn, args=(conn, addr), daemon=True).start()
    finally:
        server.close()


def main() -> None:
    """Parse command-line arguments and start the monitor."""
    parser = argparse.ArgumentParser(
        prog="controller_monitor",
        description="Decode Meta Touch controller telemetry streamed by HTS.",
    )
    parser.add_argument(
        "--protocol",
        choices=("udp", "tcp"),
        default="udp",
        help="Transport protocol to listen on (default: udp).",
    )
    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=None,
        help="Port to listen on (default: 9000 for UDP, 8000 for TCP).",
    )
    parser.add_argument(
        "--host",
        default=None,
        help="Host/IP to bind to (default: 0.0.0.0 for UDP, localhost for TCP).",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Verify the keep-alive signals (plan §10/#11): 10 Hz "
             "trigger/grip/thumbrest readout plus EDGE/CHECK lines. Squeeze the "
             "grip and press the trigger - both should track 0..1; watch whether "
             "thumb ever leaves '-' (hardware: it never did, so grip backs keep-alive).",
    )
    args = parser.parse_args()

    if args.debug:
        global _DEBUG
        _DEBUG = True
        print("debug: verify grip/trigger signals + whether thumbrest ever fires - "
              "watch EDGE/CHECK lines (Ctrl+C to quit)", flush=True)

    host = args.host or ("0.0.0.0" if args.protocol == "udp" else "localhost")
    port = args.port or (9500 if args.protocol == "udp" else 8500)

    try:
        if args.protocol == "udp":
            for line in iter_udp_lines(host, port):
                handle_line(line)
        else:
            serve_tcp(host, port)
    except KeyboardInterrupt:
        print(file=sys.stderr)


if __name__ == "__main__":
    main()
