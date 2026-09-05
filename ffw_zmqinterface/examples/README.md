# Examples — live-robot scripts over the ZMQ gateway

> **Deprecated:** `release_cube.py` and `rectangle_sweep.py` were live-robot
> scripts that gated on the v1 `/control_override` `std_msgs/Bool` latch. That
> latch is gone (v2 `OverrideCmd` is the 25-qpos joint-space command rail — no
> 0/1 bit, gateway publishes no Bool, `joy_hand`'s force-TRACK coupling
> retired). They are kept frozen under **`examples/deprecated/`** as a
> historical record of the EE-space `ControlCmd` patterns they showed (hold
> both arms + open one gripper; per-corner leash walk with dedup-safe
> re-assertion). **Do not run them on a live link**, and do not re-add
> `/control_override` to bring them back — re-derive the authority gate from
> the joint rail (§2.6 of `ffw_zmqinterface/README.md`).

Active:
  `override_round_trip_check.py` -- live-link round-trip check for the v2
  joint-space OverrideCmd rail. SUBs Obs (6001), PUBs the observed 25 joint
  positions back out as an OverrideCmd on 6002, and per received Obs diffs the
  25 sent floats against the 25 in the Obs joint block (index N == Obs joint N
  == dataset q-column N), printing max |diff| + last diff per joint. Pure ZMQ
  client -- no rclpy. Run from the package root:

      python3 examples/override_round_trip_check.py                # local gateway
      python3 examples/override_round_trip_check.py --host 192.168.0.249
      python3 examples/override_round_trip_check.py --margin 0.01 --seconds 3

  Server-side corroboration: the gateway sees BOTH the Obs it publishes
  (6001) and the OverrideCmd it receives back (6002), so its override_check
  loop check (on by default) diffs every received OverrideCmd against the
  joint block of the last Obs it sent out, WARNs past `-p override_margin`,
  and prints a per-joint table + PASS/FAIL verdict at shutdown -- the
  authoritative answer to the same question this script asks from the client.
  Run the gateway with `-p override_check:=true` and read its [override-loop]
  logs alongside this script's table.

  Note: the rail's robot-side consumer has landed (ffw_ik_solver_teleop:
  goal_source=ee|rail + apply_rail_sync() on /qpos_rail) and the gateway now
  relays OverrideCmd to /qpos_rail (dropping the 6 wheel-drive/steer names its
  MuJoCo model doesn't recognize). While goal_source stays "ee" (the launch
  default) the relay is inert -- the solver ignores /qpos_rail in EE mode --
  so this script still only validates transport + 25-for-25 ordering (Obs
  should match the sent floats when the robot holds), not robot-side tracking.
  Only flip goal_source to "rail" with hands clear of the robot and a small,
  deliberate joint delta -- that mode drives the physical arms from whatever
  the rail says. Keep hands off the SpaceMouse while it runs. The gateway (Obs
  on 6001) is already up whenever `spacemouse_unified_teleop` is running.
