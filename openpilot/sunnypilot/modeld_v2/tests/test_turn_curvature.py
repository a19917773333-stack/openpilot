from __future__ import annotations

import math

import numpy as np

from openpilot.cereal import custom
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.turn_curvature import path_curvature_for_turn


TurnDirection = custom.ModelDataV2SP.TurnDirection


def _plan(curvature: float, length: int = len(ModelConstants.T_IDXS), v_ego: float = 10.0) -> list[np.ndarray]:
  t_idxs = np.asarray(ModelConstants.T_IDXS[:length], dtype=np.float64)
  trajectory = np.zeros((length, 15), dtype=np.float32)
  x = y = yaw = 0.0
  for i, t in enumerate(t_idxs):
    if i > 0:
      dt = t - t_idxs[i - 1]
      yaw_mid = yaw + curvature * v_ego * dt / 2.0
      x += v_ego * dt * math.cos(yaw_mid)
      y += v_ego * dt * math.sin(yaw_mid)
      yaw += curvature * v_ego * dt
    trajectory[i, Plan.POSITION][0] = x
    trajectory[i, Plan.POSITION][1] = y
    trajectory[i, Plan.T_FROM_CURRENT_EULER][2] = yaw
    trajectory[i, Plan.ORIENTATION_RATE][2] = curvature * v_ego
  return [trajectory]


def _lanes(center: float = 0.0, width: float = 3.6, heading: float = 0.0,
           prob: float = 1.0) -> tuple[np.ndarray, np.ndarray]:
  xs = np.asarray(ModelConstants.X_IDXS, dtype=np.float32)
  lines = np.zeros((1, 4, len(xs), 2), dtype=np.float32)
  for i in range(4):
    lines[0, i, :, 0] = xs
  lane_center = center + np.tan(heading) * xs
  # Model y is positive to the right: left line negative, right line positive.
  lines[0, 1, :, 1] = lane_center - width / 2.0
  lines[0, 2, :, 1] = lane_center + width / 2.0
  lines[0, 0, :, 1] = lane_center - width
  lines[0, 3, :, 1] = lane_center + width
  probs = np.full((1, 8), prob, dtype=np.float32)
  return lines, probs


def test_left_turn_uses_negative_path_curvature():
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft,
  )
  assert curvature is not None
  assert curvature < -0.005


def test_right_turn_uses_positive_path_curvature():
  curvature = path_curvature_for_turn(
    _plan(0.01), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnRight,
  )
  assert curvature is not None
  assert curvature > 0.005


def test_no_turn_does_not_override_action_curvature():
  assert path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.none,
  ) is None


def test_malformed_plan_falls_back():
  assert path_curvature_for_turn(
    _plan(-0.01, length=len(ModelConstants.T_IDXS) - 1),
    ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft,
  ) is None
  assert path_curvature_for_turn(None, ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft) is None


def test_hold_prevents_mid_turn_collapse():
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.005, prev_curvature=-0.028, dt=0.05,
  )
  assert curvature is not None
  assert curvature < -0.026


def test_path_does_not_ratchet_after_turn_starts():
  curvature = path_curvature_for_turn(
    _plan(-0.08), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.01, prev_curvature=-0.02, dt=0.05,
  )
  assert curvature is not None
  assert abs(curvature) <= 0.02 + 1e-6


def test_opposite_path_releases_without_counter_steer():
  curvature = -0.028
  for _ in range(30):
    curvature = path_curvature_for_turn(
      _plan(0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
      action_curvature=0.0, prev_curvature=curvature, dt=0.05,
    )
    assert curvature is not None
    assert curvature <= 0.0
  assert curvature > -0.001


def test_lateral_acceleration_limit_caps_fast_turns():
  curvature = path_curvature_for_turn(
    _plan(-0.08), ModelConstants.T_IDXS, 8.0, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.05, prev_curvature=-0.05, dt=0.05,
  )
  assert curvature is not None
  assert abs(curvature) <= 2.5 / (8.0 * 8.0) + 1e-6


def test_camera_centered_lane_ends_turn():
  lanes, probs = _lanes()
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.04, prev_curvature=-0.04, dt=0.1,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert abs(curvature) < 0.038


def test_camera_does_not_cancel_turn_at_entry():
  lanes, probs = _lanes()
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=0.0, prev_curvature=0.0, dt=0.05,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert abs(curvature) > 0.005


def test_camera_left_line_reduces_left_turn():
  lanes, probs = _lanes(center=1.7)
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.03, prev_curvature=-0.03, dt=0.05,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert -0.02 < curvature <= 0.0


def test_camera_low_confidence_is_ignored():
  lanes, probs = _lanes(center=1.7, prob=0.1)
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.03, prev_curvature=-0.03, dt=0.05,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert abs(curvature) > 0.029


def test_camera_counter_steer_is_capped():
  lanes, probs = _lanes(center=1.7)
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.005, prev_curvature=-0.005, dt=0.05,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert 0.0 <= curvature <= 0.0101
