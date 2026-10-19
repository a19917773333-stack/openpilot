from __future__ import annotations

import math

import numpy as np

from openpilot.cereal import custom
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.turn_curvature import (
  TURN_START_ASSIST_MAX,
  _curvature_limit,
  _recorded_turn_angle_deg,
  _recorded_turn_curvature,
  path_curvature_for_turn,
)


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


def test_ordinary_low_speed_curve_is_not_treated_as_turn_release():
  assert path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 2.0, 0.5, TurnDirection.none,
    action_curvature=-0.02, prev_curvature=0.10, dt=0.05,
    steering_angle_deg=-150.0, release_turn=False,
  ) is None


def test_malformed_plan_falls_back():
  assert path_curvature_for_turn(
    _plan(-0.01, length=len(ModelConstants.T_IDXS) - 1),
    ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft,
  ) is None
  assert path_curvature_for_turn(None, ModelConstants.T_IDXS, 10.0, 0.5, TurnDirection.turnLeft) is None


def test_malformed_plan_holds_an_established_turn():
  curvature = path_curvature_for_turn(
    None, ModelConstants.T_IDXS, 2.0, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.01, prev_curvature=-0.10, dt=0.05,
  )
  assert curvature is not None
  assert curvature <= -0.09


def test_unusable_low_speed_plan_does_not_snap_to_the_action_head():
  # At walking speed the first five seconds of the plan may cover less than
  # the minimum arc used by the path estimator.  That must not release a
  # sharp turn in one frame.
  curvature = path_curvature_for_turn(
    _plan(0.0, v_ego=0.5), ModelConstants.T_IDXS, 1.5, 0.5, TurnDirection.turnRight,
    action_curvature=0.01, prev_curvature=0.10, dt=0.05,
  )
  assert curvature is not None
  assert curvature >= 0.09


def test_after_lamp_off_settles_in_the_same_direction():
  previous = 0.10
  for _ in range(80):
    result = path_curvature_for_turn(
      _plan(-0.01), ModelConstants.T_IDXS, 2.0, 0.5, TurnDirection.none,
      action_curvature=-0.02, prev_curvature=previous, dt=0.05,
      steering_angle_deg=-150.0, release_turn=True,
    )
    if result is None:
      break
    assert result >= 0.0
    assert result <= previous + 1e-6
    previous = result
  assert previous < 0.02


def test_settle_does_not_run_once_the_wheel_is_centred():
  assert path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 2.0, 0.5, TurnDirection.none,
    action_curvature=-0.02, prev_curvature=0.10, dt=0.05,
    steering_angle_deg=0.0, release_turn=True,
  ) is None


def test_hold_prevents_mid_turn_collapse():
  curvature = path_curvature_for_turn(
    _plan(-0.01), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.005, prev_curvature=-0.028, dt=0.05,
  )
  assert curvature is not None
  assert curvature < -0.026


def test_path_does_not_jump_past_recorded_floor():
  curvature = path_curvature_for_turn(
    _plan(-0.08), ModelConstants.T_IDXS, 4.2, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.01, prev_curvature=-0.02, dt=0.05,
  )
  assert curvature is not None
  assert abs(curvature) <= _recorded_turn_curvature(4.2, 12.0, 2.89) + 1e-6


def test_recorded_reference_angles_match_baseline():
  assert abs(_recorded_turn_angle_deg(4.988) - 114.0) < 0.2
  assert abs(_recorded_turn_angle_deg(2.639) - 203.6) < 0.2


def test_low_speed_limit_allows_recorded_angles():
  assert _curvature_limit(2.639) >= _recorded_turn_curvature(2.639, 12.0, 2.89) - 1e-6
  assert _curvature_limit(4.988) >= _recorded_turn_curvature(4.988, 12.0, 2.89) - 1e-6


def _hold_turn(v_ego: float, direction, action_curvature: float, frames: int = 120,
               lane_prob: float = 0.01, steering_angle_deg: float = 0.0) -> float:
  lanes, probs = _lanes(prob=lane_prob)
  curvature = 0.0
  for _ in range(frames):
    result = path_curvature_for_turn(
      _plan(-0.005 if direction == TurnDirection.turnLeft else 0.005),
      ModelConstants.T_IDXS, v_ego, 0.5, direction,
      action_curvature=action_curvature, prev_curvature=curvature, dt=0.05,
      lane_lines=lanes, lane_probs=probs, steering_angle_deg=steering_angle_deg,
    )
    assert result is not None
    curvature = result
  return curvature


def test_recorded_left_angle_is_reached_without_lane_paint():
  curvature = _hold_turn(4.988, TurnDirection.turnLeft, -0.016, steering_angle_deg=114.0)
  expected = -_recorded_turn_curvature(4.988, 12.0, 2.89)
  assert abs(curvature - expected) < 0.004


def test_recorded_right_angle_is_reached_without_lane_paint():
  curvature = _hold_turn(2.639, TurnDirection.turnRight, 0.015, steering_angle_deg=-203.6)
  expected = _recorded_turn_curvature(2.639, 12.0, 2.89)
  assert abs(curvature - expected) < 0.004


def test_recorded_floor_starts_through_entry_cap():
  lanes, probs = _lanes(prob=0.01)
  curvature = path_curvature_for_turn(
    _plan(0.005), ModelConstants.T_IDXS, 2.639, 0.5, TurnDirection.turnRight,
    action_curvature=0.0, prev_curvature=0.0, dt=0.05,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert 0.0 < curvature <= TURN_START_ASSIST_MAX + 1e-6


def test_signaled_turn_starts_from_a_straight_model_plan():
  curvature = path_curvature_for_turn(
    _plan(-0.0004), ModelConstants.T_IDXS, 4.988, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.0004, prev_curvature=0.0, dt=0.05,
    steering_angle_deg=0.0, manual_turn=True,
  )
  assert curvature is not None
  assert curvature < -0.005


def test_straight_plan_without_a_turn_request_stays_straight():
  curvature = path_curvature_for_turn(
    _plan(-0.0004), ModelConstants.T_IDXS, 4.988, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.0004, prev_curvature=0.0, dt=0.05,
    steering_angle_deg=0.0,
  )
  assert curvature is not None
  assert abs(curvature) < 0.003


def test_recorded_floor_follows_observed_angle():
  lanes, probs = _lanes(prob=0.01)
  curvature = path_curvature_for_turn(
    _plan(-0.005), ModelConstants.T_IDXS, 4.988, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.016, prev_curvature=-0.03, dt=0.05,
    lane_lines=lanes, lane_probs=probs, steering_angle_deg=57.0,
  )
  assert curvature is not None
  full = _recorded_turn_curvature(4.988, 12.0, 2.89)
  expected = full * (57.0 / 114.0 + 0.05)
  assert abs(curvature + expected) < 0.003


def test_recorded_floor_ignores_counter_steer():
  lanes, probs = _lanes(prob=0.01)
  curvature = path_curvature_for_turn(
    _plan(-0.005), ModelConstants.T_IDXS, 4.988, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.005, prev_curvature=-0.03, dt=0.05,
    lane_lines=lanes, lane_probs=probs, steering_angle_deg=-57.0,
  )
  assert curvature is not None
  assert abs(curvature) < 0.0305


def test_recorded_floor_is_disabled_when_camera_lane_is_confident():
  lanes, probs = _lanes(heading=0.5)
  curvature = path_curvature_for_turn(
    _plan(-0.005), ModelConstants.T_IDXS, 4.988, 0.5, TurnDirection.turnLeft,
    action_curvature=-0.005, prev_curvature=-0.02, dt=0.05,
    lane_lines=lanes, lane_probs=probs,
  )
  assert curvature is not None
  assert abs(curvature) < 0.02


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
