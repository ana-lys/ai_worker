#!/usr/bin/env python3
"""
Debug visualizer for scan_to_map_icp.

Live mode (ROS2 required):
  python3 debug_scan_map.py \
      --map ../ffw_mapping/all_walls_downsampled_rotated.txt

  Subscribes to /scan and /icp_pose_raw, transforms scan points
  into the map frame using the corrected pose, and updates a matplotlib
  figure interactively.

Static mode (no ROS2, just plot the map):
  python3 debug_scan_map.py \
      --map ../ffw_mapping/all_walls_downsampled_rotated.txt --static

  Plots the map points colored by wall_id and exits.

Keyboard shortcuts (live mode):
  h / H    toggle map-point highlight (click-near)
  f / F    freeze / unfreeze display
  s / S    save screenshot to debug_scan_map_screenshot.png
  r / R    reset view to data bounding box
  q        close figure
"""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

# ---------------------------------------------------------------------------
# Map loader
# ---------------------------------------------------------------------------

def load_map(path: str):
    """Load map CSV (header + wall_id x y rows). Returns (points, wall_ids)."""
    points, wall_ids = [], []
    with open(path, newline="") as f:
        reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
        header_skipped = False
        for row in reader:
            if not row or not row[0].strip() or row[0].strip().startswith("#"):
                continue
            cells = [c.strip() for c in row if c.strip()]
            if len(cells) < 3:
                continue
            if not header_skipped:
                try:
                    float(cells[0])
                except ValueError:
                    header_skipped = True
                    continue
            header_skipped = True
            wall_ids.append(int(float(cells[0])))
            points.append((float(cells[1]), float(cells[2])))
    return points, wall_ids


# ---------------------------------------------------------------------------
# Static mode — just plot the map and exit
# ---------------------------------------------------------------------------

def show_static(path: str):
    import matplotlib.pyplot as plt

    points, wall_ids = load_map(path)
    if not points:
        print("No map points loaded.", file=sys.stderr)
        return

    groups: dict[int, list] = defaultdict(list)
    for wid, pt in zip(wall_ids, points):
        groups[wid].append(pt)

    fig, ax = plt.subplots(figsize=(10, 8))
    _config_axes(ax, points)
    ax.set_title(f"Static map — {len(points)} points")

    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red",
              "tab:purple", "tab:brown", "tab:pink", "tab:gray", "tab:olive"]
    for i, (wid, pts) in enumerate(sorted(groups.items())):
        xs, ys = zip(*pts)
        ax.scatter(xs, ys, s=12, c=colors[i % len(colors)],
                   label=f"wall {wid}", zorder=3)

    ax.legend(markerscale=2)
    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Live mode — ROS2 subscriber + interactive matplotlib
# ---------------------------------------------------------------------------

def _laser_angles(msg):
    """Yield (angle, range) for every valid point in a LaserScan."""
    a = msg.angle_min
    for r in msg.ranges:
        if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
            yield a, r
        a += msg.angle_increment


def _scan_in_map_frame(msg, pose):
    """
    Transform a LaserScan into map-frame (x, y) points.

    *pose* is a geometry_msgs/Pose in the map frame (from /icp_pose_raw
    or the EKF output).
    """
    # yaw from quaternion
    q = pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)
    cx, cy = pose.position.x, pose.position.y

    out = []
    for a, r in _laser_angles(msg):
        lx = r * math.cos(a)
        ly = r * math.sin(a)
        mx = cx + lx * math.cos(yaw) - ly * math.sin(yaw)
        my = cy + lx * math.sin(yaw) + ly * math.cos(yaw)
        out.append((mx, my))
    return out


def _config_axes(ax, points):
    """Set equal aspect, grid, and a decent bounding box."""
    xs, ys = zip(*points)
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    margin = max((xmax - xmin) * 0.15, (ymax - ymin) * 0.15, 0.5)
    ax.set_xlim(xmin - margin, xmax + margin)
    ax.set_ylim(ymin - margin, ymax + margin)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)


def run_live(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from geometry_msgs.msg import Pose

    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt

    # SensorDataQoS equivalent: best-effort, keep-last 10
    _sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    # ---- load map ----
    map_points, _ = load_map(args.map)
    if not map_points:
        print("No map points loaded.", file=sys.stderr)
        sys.exit(1)
    print(f"Loaded {len(map_points)} map points.")

    # ---- figure ----
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 10))
    fig.canvas.manager.set_window_title("scan_to_map_icp debug")
    _config_axes(ax, map_points)
    ax.set_title(f"Map ({len(map_points)} pts) + live scan — {args.scan_topic} → {args.pose_topic}")

    # static scatter: map
    map_art = ax.scatter(
        [p[0] for p in map_points], [p[1] for p in map_points],
        s=8, c="tab:blue", label="map", zorder=2,
    )
    # dynamic scatter: scan (updated each frame)
    scan_art = ax.scatter(
        [], [], s=2, c="tab:orange", alpha=0.6,
        label="scan (map frame)", zorder=3,
    )
    ax.legend(markerscale=4)
    plt.tight_layout()

    # ---- state ----
    frozen = False
    # TkAgg's FigureManager doesn't have a "statusbar" attribute,
    # so just catch it silently if missing.
    _sb = getattr(fig.canvas.manager, "statusbar", None)
    def status(msg: str):
        if _sb is not None:
            try:
                _sb.set_message(msg)
            except Exception:
                pass

    # ---- update function (called from ROS callback) ----
    def update_scan(scan_pts):
        if scan_pts:
            xs, ys = zip(*scan_pts)
            scan_art.set_offsets(list(zip(xs, ys)))
        else:
            scan_art.set_offsets([])
        fig.canvas.restore_region(fig.canvas.copy_from_bbox(ax.bbox))
        ax.draw_artist(scan_art)
        fig.canvas.blit(ax.bbox)
        fig.canvas.flush_events()

    # ---- keyboard shortcuts ----
    hover_on = [False]  # mutable so the closure can toggle it
    hover_line, = ax.plot([], [], "r-", lw=1, alpha=0.6, visible=False)

    def on_key(event):
        nonlocal frozen

        if event.key in ("h", "H"):
            hover_on[0] = not hover_on[0]
            if not hover_on[0]:
                hover_line.set_visible(False)
            status("Highlight on" if hover_on[0] else "Highlight off")
            fig.canvas.draw_idle()

        elif event.key in ("f", "F"):
            frozen = not frozen
            status("Frozen" if frozen else "Live")
            fig.canvas.draw_idle()

        elif event.key in ("s", "S"):
            png = "debug_scan_map_screenshot.png"
            fig.savefig(png, dpi=200)
            status(f"Saved {png}")

        elif event.key in ("r", "R"):
            _config_axes(ax, map_points)
            fig.canvas.draw_idle()
            status("Reset view")

    fig.canvas.mpl_connect("key_press_event", on_key)

    def on_motion(event):
        if not hover_on[0] or event.inaxes is None:
            hover_line.set_visible(False)
            fig.canvas.draw_idle()
            return
        cx, cy = event.xdata, event.ydata
        best_d2, best_pt = float("inf"), None
        for mx, my in map_points:
            d2 = (mx - cx) ** 2 + (my - cy) ** 2
            if d2 < best_d2:
                best_d2, best_pt = d2, (mx, my)
        if best_pt and best_d2 < 9.0:
            mx, my = best_pt
            hover_line.set_data([cx, mx], [cy, my])
            hover_line.set_visible(True)
            status(f"pt ({mx:.2f}, {my:.2f})  dist={math.sqrt(best_d2):.2f}m")
        else:
            hover_line.set_visible(False)
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("motion_notify_event", on_motion)

    # ---- ROS node ----
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry

    class DebugNode(Node):
        def __init__(self):
            super().__init__("debug_scan_map")
            self.latest_pose = None

            self.sub_scan = self.create_subscription(
                LaserScan, args.scan_topic, self._on_scan, _sensor_qos)

            if args.pose_topic == "/icp_pose_raw":
                self.sub_pose = self.create_subscription(
                    Odometry, args.pose_topic,
                    lambda m, s=self: s.__setattr__("latest_pose", m.pose.pose), _sensor_qos)
            else:
                self.sub_pose = self.create_subscription(
                    Odometry, args.pose_topic,
                    self._pose_cb, _sensor_qos)

        def _pose_cb(self, msg):
            self.latest_pose = msg.pose.pose

        def _on_scan(self, msg):
            if frozen:
                return
            if self.latest_pose is None:
                return
            pts = _scan_in_map_frame(msg, self.latest_pose)
            if pts:
                update_scan(pts)

    rclpy.init(args=sys.argv[1:] if not args.rem_args else args.rem_args)
    node = DebugNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    status("Listening …  h=highlight  f=freeze  s=screenshot  r=reset")
    try:
        while rclpy.ok() and plt.fignum_exists(fig.number):
            executor.spin_once(timeout_sec=0.05)
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Debug visualiser for scan_to_map_icp",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--map", "-m",
                        default="/home/lys/robotis_ws/src/ai_worker/ffw_mapping/all_walls_downsampled_rotated.txt",
                        help="Path to map CSV (default: %(default)s)")
    parser.add_argument("--static", "-s", action="store_true",
                        help="Static mode: just plot the map and exit")
    parser.add_argument("--scan_topic", default="/scan",
                        help="Laser scan topic (default: %(default)s)")
    parser.add_argument("--pose_topic", default="/icp_pose_raw",
                        help="Odometry topic with corrected pose in map frame (default: %(default)s)")
    parser.add_argument("rem_args", nargs=argparse.REMAINDER,
                        help="Extra ROS2 arguments passed to rclpy.init (everything after --)")

    args = parser.parse_args()

    if not os.path.exists(args.map):
        print(f"Map file not found: {args.map}", file=sys.stderr)
        sys.exit(1)

    if args.static:
        show_static(args.map)
    else:
        # Collect extra ROS args from after "--" in argv
        run_live(args)


if __name__ == "__main__":
    main()
