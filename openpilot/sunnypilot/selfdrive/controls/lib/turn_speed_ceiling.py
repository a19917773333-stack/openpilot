"""Low-speed turn speed ceiling shared by every longitudinal backend."""

from __future__ import annotations

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.navassist.settings import NavAssistSettings, SettingsCache


# Below the 20 mph lane-change boundary this fork already treats a signal as a
# turn request.  Leave a small margin below that boundary so a normal lane
# change can never receive a turn ceiling.
TURN_PREARM_SPEED_MPS = 30.0 / 3.6
RELEASE_ACCEL_MPS2 = 1.0
TURN_STARTED_STEERING_DEG = 8.0
TURN_CENTERED_STEERING_DEG = 2.0
TURN_CENTERED_TIME_S = 0.75


class TurnSpeedCeiling:
  """Cap the speed target while a signaled or navigation turn is active.

  The ceiling only lowers the target selected from cruise/SCC/SLA/navigation.
  It never requests acceleration and never writes an actuator command, so lead
  vehicles, FCW and every existing stopping policy keep priority.
  """

  def __init__(self, settings_provider=None):
    self._settings_provider = settings_provider if settings_provider is not None else SettingsCache().read
    self.output_v_target = V_CRUISE_UNSET
    self.is_active = False
    self.is_releasing = False
    self._armed = False
    self._turn_started = False
    self._centered_time = 0.0

  @staticmethod
  def _lane_turn_active(sm) -> bool:
    service = 'modelDataV2SP'
    seen = getattr(sm, 'seen', {})
    alive = getattr(sm, 'alive', {})
    valid = getattr(sm, 'valid', {})
    if not (seen.get(service) and alive.get(service) and valid.get(service)):
      return False
    try:
      direction = sm[service].laneTurnDirection
      # Cap'n Proto readers expose an enum object, not an int; tests may pass
      # a plain numeric value.
      return int(getattr(direction, 'raw', direction)) != 0
    except (KeyError, AttributeError, TypeError, ValueError):
      return False

  def _reset(self) -> None:
    self.output_v_target = V_CRUISE_UNSET
    self.is_active = False
    self.is_releasing = False
    self._armed = False
    self._turn_started = False
    self._centered_time = 0.0

  def _release(self, v_cruise: float) -> None:
    self.is_active = False
    if self.output_v_target == V_CRUISE_UNSET:
      self.is_releasing = False
      return
    self.output_v_target = min(v_cruise, self.output_v_target + RELEASE_ACCEL_MPS2 * DT_MDL)
    self.is_releasing = self.output_v_target < v_cruise - 1e-3
    if not self.is_releasing:
      self.output_v_target = V_CRUISE_UNSET

  def update(self, sm, *, v_ego: float, v_cruise: float, long_active: bool) -> None:
    settings: NavAssistSettings = self._settings_provider()
    if not settings.turn_slowdown_enabled or not long_active:
      self._reset()
      return

    car_state = sm['carState']
    steering_angle = float(getattr(car_state, 'steeringAngleDeg', 0.0))
    one_blinker = bool(getattr(car_state, 'leftBlinker', False) != getattr(car_state, 'rightBlinker', False))
    lane_turn = self._lane_turn_active(sm)

    # A forgotten lamp must not cap the car forever.  Only release after the
    # wheel has actually turned and then stayed centred with the lamp still on.
    if abs(steering_angle) >= TURN_STARTED_STEERING_DEG:
      self._turn_started = True
    if abs(steering_angle) <= TURN_CENTERED_STEERING_DEG:
      self._centered_time += DT_MDL
    else:
      self._centered_time = 0.0

    if not one_blinker and not lane_turn:
      self._armed = False
    elif self._turn_started and self._centered_time >= TURN_CENTERED_TIME_S:
      self._armed = False
    elif one_blinker and v_ego <= TURN_PREARM_SPEED_MPS:
      self._armed = True
    elif lane_turn:
      self._armed = True

    if not self._armed:
      self._turn_started = False
      self._centered_time = 0.0
      self._release(v_cruise)
      return

    self.is_active = True
    self.is_releasing = False
    self.output_v_target = min(v_cruise, settings.turn_speed_kph / 3.6)
