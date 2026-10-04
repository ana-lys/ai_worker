"""Arm joint effort: raw /joint_states -> amps -> joint torque, one model for sim and real.

Obs.joint_effort carries amps on every arm joint. The robot's /joint_states does
not: the driver reports YM080 current / 13 and YM070 current / 25 in 0.01 A
counts, and the PH42 on J7 in 1 mA counts. raw_to_amps() undoes that (the same
scales as fit_contact_model.EFF_TO_A); the gateway applies it before encoding
Obs, so Obs effort is A on both backends.

Torque per amp is the load-cell joint model's DRIVING K (fit_contact_model.py
fit_joint_model, 698 presses, 2026-09-28), per arm and joint: K_drive = K * eta,
the gear efficiency eta ~0.3-0.8 on J1-J5. Driving is the state that matters
here -- pushing the peg, the motor drives the load. One constant K per joint, not
switched by motion state: the arm holds tens of N m of gravity, and switching K
when a joint starts moving turns that gravity current into fake ~10 N force
jumps (2026-10-04 replay: free-air sd 1.5 N switched vs 0.46 N constant). With a
constant K, gravity cancels exactly against a baseline. J6 fits inverted (eta > 1)
and the right J7 is unidentified, so J6/J7 use their single-K fit. The sim reports
amps with amps_from_torque(), the exact inverse.
"""
import numpy as np

ARMS = ("right", "left")
EFF_TO_A = np.array([13 * 0.01] * 3 + [25 * 0.01] * 3 + [0.001])      # raw /joint_states -> A

# N m / A, joints 1..7: driving K (J1-J5), single-K fit (J6, J7)
K_DRIVE = {"right": np.array([6.46, 3.66, 3.58, 2.00, 2.45, 7.19, 17.29]),
           "left": np.array([4.93, 3.68, 2.67, 2.08, 2.51, 5.73, 13.64])}
# gear efficiency of the same fit (J6/J7 = 1), for reference: K = K_DRIVE / ETA
ETA = {"right": np.array([0.80, 0.70, 0.55, 0.30, 0.55, 1.0, 1.0]),
       "left": np.array([0.71, 0.66, 0.41, 0.48, 0.48, 1.0, 1.0])}
# per-joint torque noise of the fit (stiction), weights the J^T solve [N m]
SIGMA = np.array([9.9, 8.2, 11.3, 8.2, 3.8, 5.4, 4.5])


def arm_joint_index(name):
    """(arm, 0-based joint index) for arm_<l|r>_joint<n>, else None."""
    if not name.startswith("arm_") or "_joint" not in name:
        return None
    return ("left" if name[4] == "l" else "right"), int(name[-1]) - 1


def raw_to_amps(name, raw):
    """One /joint_states effort value -> A. Non-arm joints pass through unchanged."""
    aj = arm_joint_index(name)
    return float(raw) if aj is None else float(raw) * EFF_TO_A[aj[1]]


def torque_from_amps(arm, amps, j=None):
    """Joint torques [N m] from currents [A]: all 7 joints of `arm`, or joint index j."""
    return K_DRIVE[arm][slice(None) if j is None else j] * np.asarray(amps, float)


def amps_from_torque(arm, tau, j=None):
    """Inverse of torque_from_amps."""
    return np.asarray(tau, float) / K_DRIVE[arm][slice(None) if j is None else j]
