"""Peg-in-hole environment core for the FFW dual-arm robot (no ROS, no gym).

Server-side functions that a ZMQ server calls when HIL-SERL commands arrive:
reset(seed) and step(delta), on a MuJoCo backend (lockstep, as fast as the CPU
allows) or, later, the real robot (15 Hz).
"""
import os

# Headless GPU rendering by default (no display needed); device 0 = the NVIDIA
# GPU on the robot workstation (device 1 is a software driver that hangs).
# Must be set before mujoco creates a GL context; MUJOCO_GL=glfw overrides.
os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("MUJOCO_EGL_DEVICE_ID", "0")

from .peg_hole import PegHoleConfig, PegHoleTask  # noqa: E402
from .sim_backend import MujocoBackend  # noqa: E402
from .solver import InsertionSolver  # noqa: E402

__all__ = ["PegHoleConfig", "PegHoleTask", "MujocoBackend", "InsertionSolver"]
