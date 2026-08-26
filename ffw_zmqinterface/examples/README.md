# Examples — driving the arm over the ZMQ gateway

These are **live-robot demos**: they command the real robot through the framed
ZMQ link (`tcp://127.0.0.1:6002`, port 6001/6002 from `ffw_zmqinterface/README.md`).
They are hybrid ROS + ZMQ — the goal goes out over ZMQ, while monitoring and
the `/control_override` gate use ROS topics.

> **Run only with the stack up on `ROS_DOMAIN_ID=30` and the gateway reachable
> on `127.0.0.1:6002`, with nobody's hands near the arm / off the SpaceMouse.**
> `/control_override` is engaged for the whole run and released at the end.

## `rectangle_sweep.py` — "move the arm in a square"

Right-EE rectangle sweep in the map frame: x 0.45..0.60, y 0..-0.30, 10 loops.
z/roll/pitch/yaw are held at the start pose; the left arm is locked at its start
pose. Every `ControlCmd` carries a fresh header timestamp so each re-assertion
passes the gateway dedup and the solver's leash walks the arm to the current
corner; once within `CORNER_TOL`, it advances.

Requirements: `pyzmq`, `rclpy`, `geometry_msgs`, `std_msgs`, the running stack.

```
ROS_DOMAIN_ID=30 python3 examples/rectangle_sweep.py
```

Prints one line per corner (`reached in …` / `MISS …`), writes the achieved
path to `/tmp/rectangle_path.csv`, and exits `PASS`/`FAIL` based on whether
every corner was reached. Tune the geometry with the `X_MIN/X_MAX/Y_MIN/Y_MAX`,
`LOOPS`, `CORNER_TOL`, `CORNER_TIMEOUT` constants at the top of the file.
