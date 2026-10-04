"""Scripted insertion from the true peg and hole poses (sim pipeline check, and
the push assist later).

Two phases, each a pose target for the peg tool site, converted to the EE
target and turned into a 6-DOF base_link delta from the current EE command:
  ALIGN    peg on the hole axis, aligned, tip `hover` above the rim
  INSERT   once aligned within `align_tol`, slide down the axis to
           `goal_depth` below the rim (default success depth + 4 mm), staying on the axis
"""
import numpy as np

from .geometry import delta_between, pose_error
from .peg_hole import rel_aligned


class InsertionSolver:
    def __init__(self, task, hover=0.005, goal_depth=None, align_tol=(0.0003, np.radians(0.3)),
                 descend_step=0.004):
        self.task = task
        self.hover = hover
        self.goal_depth = task.cfg.success_depth + 0.004 if goal_depth is None else goal_depth
        self.align_tol = align_tol
        self.descend_step = descend_step
        self.phase = "ALIGN"

    def reset(self):
        self.phase = "ALIGN"

    def action(self):
        t = self.task
        hole = t.b.tool_pose("left")
        roll = t.cfg.roll_deg
        if self.phase == "ALIGN":
            target = hole @ rel_aligned(t.peg_site_height(self.hover), roll)
            pe, re = pose_error(t.b.tool_pose("right"), target)
            if pe < self.align_tol[0] and re < self.align_tol[1]:
                self.phase = "INSERT"
        if self.phase == "INSERT":
            s_now = t.insertion_state()["height"]
            s_goal = t.peg_site_height(-self.goal_depth)
            target = hole @ rel_aligned(max(s_goal, s_now - self.descend_step), roll)
        # steer from the observed EE (target - observed), so tracking offsets are
        # integrated away through the command
        return delta_between(t.b.site_pose("right"), t.to_ee("right", target))
