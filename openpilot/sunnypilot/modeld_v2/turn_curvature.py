from __future__ import annotations

import math

from openpilot.cereal import custom
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.sunnypilot.modeld_v2.constants import Plan


TurnDirection = custom.ModelDataV2SP.TurnDirection

# Openpilot's angle controller negates curvature into a left-positive steering
# angle, so a left maneuver asks for negative curvature and a right for positive.
TURN_CURVATURE_SIGN = {int(TurnDirection.turnLeft): -1.0, int(TurnDirection.turnRight): 1.0}
OPPOSITE_CURVATURE_EPS = 1e-4


def turn_sign(turn_direction) -> float | None:
  return TURN_CURVATURE_SIGN.get(int(getattr(turn_direction, "raw", turn_direction)))


def path_curvature_for_turn(plan, t_idxs, v_ego: float, action_t: float, turn_direction) -> float | None:
  """Curvature of the model's own path at the actuation delay during a turn.

  The action head can lag the predicted path when a turn starts. This samples
  the same trajectory the display draws at the near end (the existing latency
  horizon) instead of the far end of the plan. Returns None when there is no
  turn, no usable plan, or the path curves away from the requested direction;
  callers then keep their normal action-head curvature.
  """
  sign = turn_sign(turn_direction)
  if sign is None or plan is None or len(plan) == 0:
    return None
  trajectory = plan[0]
  if trajectory.shape[0] != len(t_idxs):
    return None
  curvature = float(get_curvature_from_plan(
    trajectory[:, Plan.T_FROM_CURRENT_EULER][:, 2],
    trajectory[:, Plan.ORIENTATION_RATE][:, 2],
    t_idxs, v_ego, action_t,
  ))
  if not math.isfinite(curvature) or curvature * sign < -OPPOSITE_CURVATURE_EPS:
    return None
  return curvature
