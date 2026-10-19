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

# Once the lamp turns off the car can still be unwinding from a sharp low-speed
# turn.  Do not hand control straight back to the action head while its lane
# confidence is poor: hold the established direction and release it at a
# bounded rate until the wheel is close to centre.
TURN_SETTLE_MIN_CURVATURE = 0.015
TURN_SETTLE_MIN_ANGLE_DEG = 8.0
TURN_SETTLE_MAX_SPEED_MPS = 8.0
TURN_SETTLE_RELEASE_RATE = 0.03  # 1/m per second

MAX_TURN_CURVATURE = 0.12      # 1/m, radius 8.3 m
MAX_TURN_LATERAL_ACCEL = 2.5   # m/s^2

# Baseline recorded from route 000000fb--9b2ec00d7f on 2026-10-19 with the
# driver demonstrating the required low-speed turn angle.  The model plan has
# almost no usable lane paint at both turn locations, so its action curvature
# collapses well below the demonstrated angle.  Interpolating angle against
# 1/v reproduces the two recorded operating points and naturally softens the
# target as speed rises.
TURN_REFERENCE_SPEEDS = (2.639, 4.988)
TURN_REFERENCE_ANGLES_DEG = (203.6, 114.0)
TURN_REFERENCE_MAX_ANGLE_DEG = 220.0
TURN_REFERENCE_MIN_SPEED_MPS = 1.0
TURN_REFERENCE_PROGRESS_LEAD = 0.05
TURN_MANUAL_START_PROGRESS = 0.30
DEFAULT_STEER_RATIO = 12.0
DEFAULT_WHEELBASE_M = 2.89

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


def _recorded_turn_angle_deg(v_ego: float) -> float:
  """Return the steering-wheel angle demonstrated by the driver."""
  v_ego = max(float(v_ego), TURN_REFERENCE_MIN_SPEED_MPS)
  inv_v = 1.0 / v_ego
  v_slow, v_fast = TURN_REFERENCE_SPEEDS
  angle_slow, angle_fast = TURN_REFERENCE_ANGLES_DEG
  inv_slow, inv_fast = 1.0 / v_slow, 1.0 / v_fast
  angle = angle_slow + (angle_fast - angle_slow) * (inv_v - inv_slow) / (inv_fast - inv_slow)
  return float(np.clip(angle, 0.0, TURN_REFERENCE_MAX_ANGLE_DEG))


def _recorded_turn_curvature(v_ego: float, steer_ratio: float,
                             wheelbase: float) -> float:
  """Convert the recorded wheel angle to road curvature."""
  if not (math.isfinite(steer_ratio) and math.isfinite(wheelbase)):
    return 0.0
  if steer_ratio <= 0.0 or wheelbase <= 0.0:
    return 0.0
  road_angle = math.radians(_recorded_turn_angle_deg(v_ego)) / steer_ratio
  return math.tan(road_angle) / wheelbase


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
                            lane_lines=None, lane_probs=None,
                            steering_angle_deg: float = 0.0,
                            steer_ratio: float = DEFAULT_STEER_RATIO,
                            wheelbase: float = DEFAULT_WHEELBASE_M,
                            manual_turn: bool = False,
                            release_turn: bool = False) -> float | None:
  """Turn curvature that cannot snap back to straight in the middle of a turn.

  Returns None outside a turn or with an unusable plan so the caller keeps its
  normal action-head behaviour.  During a turn the returned magnitude is
  rate-limited: it may rise quickly, but while the model plan still points into
  the turn it may only unwind slowly.  A plan that has straightened or turned
  the other way releases quickly and is never allowed to command a counter
  steer while the lamp is still asking for the turn.
  """
  sign = turn_sign(turn_direction)
  settling = False
  if sign is None:
    # No lamp request remains, but a just-completed sharp turn can still be
    # unwinding.  release_turn is supplied by modeld only after a real turn
    # request ended, so ordinary low-speed curves keep normal lane following.
    previous = float(prev_curvature)
    if (release_turn and abs(previous) >= TURN_SETTLE_MIN_CURVATURE and
        abs(float(steering_angle_deg)) >= TURN_SETTLE_MIN_ANGLE_DEG and
        v_ego <= TURN_SETTLE_MAX_SPEED_MPS):
      sign = 1.0 if previous > 0.0 else -1.0
      settling = True
    else:
      return None

  t_idxs = np.asarray(t_idxs, dtype=np.float64)
  path_curvature = None
  path_continues = False
  if plan is not None and len(plan) > 0:
    trajectory = plan[0]
    if trajectory.ndim == 2 and trajectory.shape[0] == len(t_idxs) and trajectory.shape[1] >= 15:
      path_curvature, path_continues, path_usable = _path_curvature(plan, t_idxs, sign)
      if not path_usable:
        # Keep the old near-horizon fallback for models whose plan has no
        # usable position samples, but do not use it as the main low-speed
        # estimator.
        try:
          path_curvature = float(get_curvature_from_plan(
            trajectory[:, Plan.T_FROM_CURRENT_EULER][:, 2],
            trajectory[:, Plan.ORIENTATION_RATE][:, 2],
            t_idxs, v_ego, action_t,
          ))
        except (ValueError, IndexError):
          path_curvature = None
        if path_curvature is not None and not math.isfinite(path_curvature):
          path_curvature = None
        if path_curvature is not None:
          if path_curvature * sign < -OPPOSITE_CURVATURE_EPS:
            path_curvature = 0.0
            path_continues = False
          else:
            path_continues = abs(path_curvature) > 0.003

  if path_curvature is None:
    # A turn request is still active.  If the plan briefly becomes unusable
    # (common at an intersection with no paint), hold the established command
    # and release it through the normal rate limiter below.  Returning None
    # here would let the action head snap the curvature and steering angle the
    # other way in a single frame.
    if abs(prev_curvature) >= TURN_START_EPS:
      path_curvature = float(prev_curvature)
      path_continues = True
    elif settling:
      path_curvature = 0.0
    elif not manual_turn:
      return None
    else:
      path_curvature = 0.0

  action_mag = max(float(action_curvature) * sign, 0.0)
  prev_mag = max(float(prev_curvature) * sign, 0.0)
  path_mag = max(path_curvature * sign, 0.0)

  # The camera's lane lines are the primary evidence near the end of a turn.
  # When they are not trustworthy (common at an intersection or in a car park),
  # use the recorded driver angle as a floor so the action head cannot collapse
  # to a shallow arc while the plan still says the turn continues.
  lookahead = float(np.clip(v_ego * 2.0, 4.0, 10.0))
  lane = _lane_state(lane_lines, lane_probs, lookahead)

  target_mag = 0.0 if settling else action_mag
  if path_continues and path_mag > 0.0 and prev_mag <= TURN_START_EPS:
    # At entry the action head can lag the visible path.  Use the path only to
    # get the turn moving, with a hard cap.  Once a turn is established the
    # action head and the held previous value own the magnitude; otherwise a
    # noisy far-horizon path would ratchet the wheel tighter every frame.
    target_mag = max(target_mag, min(path_mag, TURN_START_ASSIST_MAX))
  limit = _curvature_limit(v_ego)
  wheel_angle = float(steering_angle_deg)
  # Wheel angle is left-positive while turn curvature is left-negative.  Do
  # not use a driver's counter-steer as evidence that the maneuver is
  # progressing.
  turn_progress_angle = abs(wheel_angle) if wheel_angle * -sign > 0.0 else 0.0
  # A driver lamp alone starts the turn, so there is no observed wheel angle
  # to follow at entry.  Begin at a fixed fraction of the recorded angle and
  # let the measured wheel angle take over as the car follows the path.
  if (not settling and lane is None and v_ego >= TURN_REFERENCE_MIN_SPEED_MPS and
      (path_continues or manual_turn)):
    reference_angle = _recorded_turn_angle_deg(v_ego)
    progress = turn_progress_angle / max(reference_angle, 1.0) + TURN_REFERENCE_PROGRESS_LEAD
    if manual_turn:
      progress = max(progress, TURN_MANUAL_START_PROGRESS)
    progress = min(1.0, progress)
    reference_mag = _recorded_turn_curvature(v_ego, steer_ratio, wheelbase) * progress
    target_mag = max(target_mag, min(reference_mag, limit))
  target_mag = min(target_mag, limit)
  prev_mag = min(prev_mag, limit)

  if not math.isfinite(target_mag) or not math.isfinite(prev_mag):
    return None

  dt = float(np.clip(dt, 0.01, 0.1))
  if target_mag >= prev_mag:
    if prev_mag <= 0.0:
      held_mag = min(target_mag, TURN_START_ASSIST_MAX)
    else:
      held_mag = min(target_mag, prev_mag + TURN_ATTACK_RATE * dt)
  else:
    if settling:
      release_rate = TURN_SETTLE_RELEASE_RATE
    else:
      release_rate = TURN_HOLD_RELEASE_RATE if path_continues else TURN_FAST_RELEASE_RATE
    held_mag = max(target_mag, prev_mag - release_rate * dt)

  # The main camera's lane lines decide whether the new lane is actually
  # reached.  A car that is aligned and centred in a confident lane is done
  # turning even if the far end of the model plan still drifts left/right.
  lane_correction = 0.0
  if lane is not None and not settling:
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
