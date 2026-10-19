from __future__ import annotations

import numpy as np

from openpilot.cereal import custom
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.turn_curvature import path_curvature_for_turn


TurnDirection = custom.ModelDataV2SP.TurnDirection


def _plan(yaw_sign: float, length: int = len(ModelConstants.T_IDXS)) -> list[np.ndarray]:
  trajectory = np.zeros((length, 15), dtype=np.float32)
  t_idxs = np.asarray(ModelConstants.T_IDXS[:length])
  trajectory[:, Plan.T_FROM_CURRENT_EULER][:, 2] = yaw_sign * 0.1 * t_idxs
  return [trajectory]


def test_left_turn_uses_negative_path_curvature():
  curvature = path_curvature_for_turn(
    _plan(-1.0), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft,
  )
  assert curvature is not None
  assert curvature < 0


def test_right_turn_uses_positive_path_curvature():
  curvature = path_curvature_for_turn(
    _plan(1.0), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnRight,
  )
  assert curvature is not None
  assert curvature > 0


def test_opposite_path_falls_back():
  assert path_curvature_for_turn(
    _plan(1.0), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft,
  ) is None


def test_no_turn_does_not_override_action_curvature():
  assert path_curvature_for_turn(
    _plan(-1.0), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.none,
  ) is None


def test_malformed_plan_falls_back():
  assert path_curvature_for_turn(
    _plan(-1.0, length=len(ModelConstants.T_IDXS) - 1),
    ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft,
  ) is None
  assert path_curvature_for_turn(None, ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft) is None
