from __future__ import annotations

import math

import numpy as np

from openpilot.cereal import custom
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.sunnypilot.modeld_v2.constants import Plan


TurnDirection = custom.ModelDataV2SP.TurnDirection

# Openpilot's angle controller negates curvature into a left-positive steering
# angle, so a left maneuver asks for negative curvature and a right for positive.
TURN_CURVATURE_SIGN = {int(TurnDirection.turnLeft): -1.0, int(TurnDirection.turnRight): 1.0}
OPPOSITE_CURVATURE_EPS = 1e-4

# Low-speed turns need a longer horizon than the actuation delay: at 4 m/s a
# 0.5 s lookahead is only two metres and the near end of the plan looks almost
# straight even while the road keeps bending.  Use the plan out to five seconds
# to decide whether the turn is still continuing, then hold the already
# established curvature instead of letting the action head collapse to zero.
TURN_MIN_HORIZON = 0.8
TURN_MAX_HORIZON = 5.0
TURN_MIN_ARC_M = 3.0
TURN_CONTINUE_YAW_RAD = 0.03

TURN_ATTACK_RATE = 0.12        # 1/m per second
TURN_HOLD_RELEASE_RATE = 0.008 # 1/m per second while the plan still turns
TURN_FAST_RELEASE_RATE = 0.04  # 1/m per second once the turn has ended
TURN_START_EPS = 0.005         # below this the turn has not really started
TURN_START_ASSIST_MAX = 0.03   # cap the path's help at turn entry

MAX_TURN_CURVATURE = 0.08      # 1/m, radius 12.5 m
MAX_TURN_LATERAL_ACCEL = 2.5   # m/s^2

LANE_MIN_PROB = 0.5
LANE_MIN_WIDTH_M = 2.5
LANE_MAX_WIDTH_M = 4.5
LANE_COMPLETE_HEADING_RAD = 0.06
LANE_COMPLETE_OFFSET_M = 0.35
LANE_COMPLETE_MIN_CURVATURE = 0.012
LANE_EDGE_MARGIN_M = 0.20
LANE_CORRECTION_MAX = 0.02
LANE_CENTER_ASSIST_MAX = 0.01
LANE_COUNTER_STEER_MAX = 0.01


def turn_sign(turn_direction) -> float | None:
  return TURN_CURVATURE_SIGN.get(int(getattr(turn_direction, "raw", turn_direction)))


def _path_curvature(plan, t_idxs: np.ndarray, sign: float) -> tuple[float | None, bool, bool]:
  """Return the plan's far-horizon curvature and whether it keeps turning.

  The plan position is in the current ego frame, so the cumulative distance
  along the first five seconds is the arc length.  Tangent yaw divided by arc
  length is the path curvature; unlike the instantaneous action head it does
  not disappear when the car is at the apex of a low-speed turn.
  """
  trajectory = plan[0]
  if trajectory.ndim != 2 or trajectory.shape[0] != len(t_idxs) or trajectory.shape[1] < 15:
    return None, False, False

  xy = np.asarray(trajectory[:, Plan.POSITION][:, :2], dtype=np.float64)
  if not np.isfinite(xy).all():
    return None, False, False

  seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
  arc = np.concatenate(([0.0], np.cumsum(seg)))
  yaws = np.asarray(trajectory[:, Plan.T_FROM_CURRENT_EULER][:, 2], dtype=np.float64)
  if not np.isfinite(yaws).all():
    return None, False, False

  # Remove any constant yaw offset in the model frame.
  yaw_delta = yaws - yaws[0]
  mask = ((t_idxs >= TURN_MIN_HORIZON) & (t_idxs <= TURN_MAX_HORIZON) &
          (arc >= TURN_MIN_ARC_M))
  if not mask.any():
    return None, False, True

  samples = yaw_delta[mask] / arc[mask]
  samples = samples[np.isfinite(samples) & (samples * sign > 0.0)]
  if samples.size == 0:
    # The plan is usable but no longer bends into this turn.  This is the
    # end-of-turn case, not a reason to snap the wheel straight in one frame.
    return 0.0, False, True

  # A median rejects the polynomial/plan wobble that can put one sample far
  # outside the actual turn radius.
  curvature = float(np.median(samples))
  far_yaw = float(yaw_delta[mask][-1])
  return curvature, far_yaw * sign > TURN_CONTINUE_YAW_RAD, True


def _curvature_limit(v_ego: float) -> float:
  lateral_limit = MAX_TURN_LATERAL_ACCEL / max(v_ego * v_ego, 1.0)
  return max(0.0, min(MAX_TURN_CURVATURE, lateral_limit))


def _line_y(line, x_query: float) -> float | None:
  points = np.asarray(line, dtype=np.float64)
  if points.ndim != 2 or points.shape[1] < 2:
    return None
  xs = points[:, 0]
  ys = points[:, 1]
  valid = np.isfinite(xs) & np.isfinite(ys) & (xs >= 0.0)
  if valid.sum() < 2:
    return None
  xs = xs[valid]
  ys = ys[valid]
  order = np.argsort(xs)
  xs = xs[order]
  ys = ys[order]
  if x_query < xs[0] - 0.5 or x_query > xs[-1] + 0.5:
    return None
  return float(np.interp(x_query, xs, ys))


def _lane_state(lane_lines, lane_probs, lookahead_m: float):
  """Read the main-camera lane lines around the car.

  Returns (centre_at_car, centre_at_lookahead, width_at_car,
  width_at_lookahead, lane_heading).  The plan lane lines use y positive to
  the left, so a car sitting left of the lane centre has a negative centre
  offset.  Only high-confidence, normal-width lanes are used.
  """
  if lane_lines is None or lane_probs is None:
    return None
  lines = np.asarray(lane_lines, dtype=np.float64)
  if lines.ndim != 4 or lines.shape[0] < 1 or lines.shape[1] < 3:
    return None

  probs = np.asarray(lane_probs, dtype=np.float64)
  if probs.ndim == 2:
    probs = probs[0, 1::2] if probs.shape[1] >= 8 else probs[0]
  else:
    probs = probs.reshape(-1)
  if probs.size < 3 or probs[1] < LANE_MIN_PROB or probs[2] < LANE_MIN_PROB:
    return None

  left = lines[0, 1]
  right = lines[0, 2]
  left0 = _line_y(left, 0.0)
  right0 = _line_y(right, 0.0)
  left_ahead = _line_y(left, lookahead_m)
  right_ahead = _line_y(right, lookahead_m)
  if None in (left0, right0, left_ahead, right_ahead):
    return None

  # Model lane-line y is positive to the right: index 1 (left) is negative,
  # index 2 (right) is positive.
  width0 = right0 - left0
  width_ahead = right_ahead - left_ahead
  if not (LANE_MIN_WIDTH_M <= width0 <= LANE_MAX_WIDTH_M and
          LANE_MIN_WIDTH_M <= width_ahead <= LANE_MAX_WIDTH_M):
    return None

  center0 = 0.5 * (left0 + right0)
  center_ahead = 0.5 * (left_ahead + right_ahead)
  heading = math.atan2(center_ahead - center0, lookahead_m)
  return center0, center_ahead, width0, width_ahead, heading


def path_curvature_for_turn(plan, t_idxs, v_ego: float, action_t: float, turn_direction,
                            action_curvature: float = 0.0,
                            prev_curvature: float = 0.0,
                            dt: float = 0.05,
                            lane_lines=None, lane_probs=None) -> float | None:
  """Turn curvature that cannot snap back to straight in the middle of a turn.

  Returns None outside a turn or with an unusable plan so the caller keeps its
  normal action-head behaviour.  During a turn the returned magnitude is
  rate-limited: it may rise quickly, but while the model plan still points into
  the turn it may only unwind slowly.  A plan that has straightened or turned
  the other way releases quickly and is never allowed to command a counter
  steer while the lamp is still asking for the turn.
  """
  sign = turn_sign(turn_direction)
  if sign is None or plan is None or len(plan) == 0:
    return None

  t_idxs = np.asarray(t_idxs, dtype=np.float64)
  trajectory = plan[0]
  if trajectory.ndim != 2 or trajectory.shape[0] != len(t_idxs) or trajectory.shape[1] < 15:
    return None

  path_curvature, path_continues, path_usable = _path_curvature(plan, t_idxs, sign)
  if not path_usable:
    # Keep the old near-horizon fallback for models whose plan has no usable
    # position samples, but do not use it as the main low-speed estimator.
    try:
      path_curvature = float(get_curvature_from_plan(
        trajectory[:, Plan.T_FROM_CURRENT_EULER][:, 2],
        trajectory[:, Plan.ORIENTATION_RATE][:, 2],
        t_idxs, v_ego, action_t,
      ))
    except (ValueError, IndexError):
      return None
    if not math.isfinite(path_curvature):
      return None
    if path_curvature * sign < -OPPOSITE_CURVATURE_EPS:
      path_curvature = 0.0
      path_continues = False
    else:
      path_continues = abs(path_curvature) > 0.003
  elif path_curvature is None:
    return None

  action_mag = max(float(action_curvature) * sign, 0.0)
  prev_mag = max(float(prev_curvature) * sign, 0.0)
  path_mag = max(path_curvature * sign, 0.0)

  target_mag = action_mag
  if path_continues and path_mag > 0.0 and prev_mag <= TURN_START_EPS:
    # At entry the action head can lag the visible path.  Use the path only to
    # get the turn moving, with a hard cap.  Once a turn is established the
    # action head and the held previous value own the magnitude; otherwise a
    # noisy far-horizon path would ratchet the wheel tighter every frame.
    target_mag = max(target_mag, min(path_mag, TURN_START_ASSIST_MAX))
  limit = _curvature_limit(v_ego)
  target_mag = min(target_mag, limit)
  prev_mag = min(prev_mag, limit)

  if not math.isfinite(target_mag) or not math.isfinite(prev_mag):
    return None

  dt = float(np.clip(dt, 0.01, 0.1))
  if target_mag >= prev_mag:
    held_mag = target_mag if prev_mag <= 0.0 else min(target_mag, prev_mag + TURN_ATTACK_RATE * dt)
  else:
    release_rate = TURN_HOLD_RELEASE_RATE if path_continues else TURN_FAST_RELEASE_RATE
    held_mag = max(target_mag, prev_mag - release_rate * dt)

  # The main camera's lane lines decide whether the new lane is actually
  # reached.  A car that is aligned and centred in a confident lane is done
  # turning even if the far end of the model plan still drifts left/right.
  lookahead = float(np.clip(v_ego * 2.0, 4.0, 10.0))
  lane = _lane_state(lane_lines, lane_probs, lookahead)
  lane_correction = 0.0
  if lane is not None:
    center0, center_ahead, width0, _, lane_heading = lane
    straddling = abs(center0) > max(0.0, width0 / 2.0 - LANE_EDGE_MARGIN_M)
    # At turn entry the car is still centred and aligned in the old lane, so
    # the camera would otherwise declare the turn complete before it starts.
    # Only let the camera finish a turn that has actually established some
    # curvature; then a centred, aligned new lane is a reliable endpoint.
    lane_complete = (prev_mag >= LANE_COMPLETE_MIN_CURVATURE and not straddling and
                     abs(lane_heading) < LANE_COMPLETE_HEADING_RAD and
                     abs(center0) < LANE_COMPLETE_OFFSET_M)
    if lane_complete:
      held_mag = max(0.0, prev_mag - TURN_FAST_RELEASE_RATE * dt)

    # Do not fight a lane that is itself still bending sharply.  Near the end
    # of the turn the heading is small and the correction can centre the car
    # without cutting the corner.
    if abs(lane_heading) < 0.5:
      if straddling:
        lane_correction = 2.0 * center0 / (lookahead * lookahead)
        lane_correction = float(np.clip(lane_correction, -LANE_CORRECTION_MAX, LANE_CORRECTION_MAX))
      elif abs(center_ahead) > 0.3:
        lane_correction = 2.0 * center_ahead / (lookahead * lookahead)
        lane_correction = float(np.clip(lane_correction, -LANE_CENTER_ASSIST_MAX, LANE_CENTER_ASSIST_MAX))

  combined = sign * held_mag + lane_correction
  if combined * sign < 0.0:
    # Lane containment may reduce the turn, but a small counter-steer is only
    # allowed when the camera says the car is already on the line.
    combined = -sign * min(abs(combined), LANE_COUNTER_STEER_MAX)
  combined = float(np.clip(combined, -limit, limit))

  if not math.isfinite(combined):
    return None
  return combined
