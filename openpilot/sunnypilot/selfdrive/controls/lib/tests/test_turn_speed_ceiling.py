from types import SimpleNamespace

from openpilot.selfdrive.car.cruise import V_CRUISE_UNSET
from openpilot.sunnypilot.navassist.settings import NavAssistSettings
from openpilot.sunnypilot.selfdrive.controls.lib.turn_speed_ceiling import (
  RELEASE_ACCEL_MPS2,
  TURN_PREARM_SPEED_MPS,
  TurnSpeedCeiling,
)


DT = 0.05


class FakeSM(dict):
  def __init__(self, *, left_blinker=False, right_blinker=False, steering=0.0,
               lane_turn=0, healthy=True):
    super().__init__(
      carState=SimpleNamespace(leftBlinker=left_blinker, rightBlinker=right_blinker,
                               steeringAngleDeg=steering),
      modelDataV2SP=SimpleNamespace(laneTurnDirection=lane_turn),
    )
    self.seen = {'modelDataV2SP': healthy}
    self.alive = {'modelDataV2SP': healthy}
    self.valid = {'modelDataV2SP': healthy}


def ceiling(**settings):
  values = NavAssistSettings(**settings)
  return TurnSpeedCeiling(settings_provider=lambda: values)


def update(controller, sm, *, speed=8.0, cruise=20.0, long_active=True):
  controller.update(sm, v_ego=speed, v_cruise=cruise, long_active=long_active)


def test_signaled_right_turn_caps_the_target_at_18_kph():
  controller = ceiling()
  update(controller, FakeSM(right_blinker=True))
  assert controller.is_active
  assert controller.output_v_target == 5.0


def test_signal_arms_before_the_car_moves_from_a_stop():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True), speed=0.0)
  assert controller.is_active
  assert controller.output_v_target == 5.0


def test_ceiling_never_raises_a_lower_existing_target():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True), cruise=3.0)
  assert controller.output_v_target == 3.0


def test_signal_above_the_lane_change_boundary_is_transparent():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True), speed=TURN_PREARM_SPEED_MPS + 0.5)
  assert not controller.is_active
  assert controller.output_v_target == V_CRUISE_UNSET


def test_two_signals_do_not_arm_the_ceiling():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True, right_blinker=True))
  assert controller.output_v_target == V_CRUISE_UNSET


def test_navigation_turn_caps_without_a_physical_lamp():
  controller = ceiling()
  update(controller, FakeSM(lane_turn=1))
  assert controller.output_v_target == 5.0


def test_capnp_enum_reader_is_accepted():
  controller = ceiling()
  sm = FakeSM()
  sm['modelDataV2SP'].laneTurnDirection = SimpleNamespace(raw=1)
  update(controller, sm)
  assert controller.output_v_target == 5.0


def test_unhealthy_model_cannot_create_a_navigation_turn():
  controller = ceiling()
  update(controller, FakeSM(lane_turn=1, healthy=False))
  assert controller.output_v_target == V_CRUISE_UNSET


def test_physical_lamp_does_not_depend_on_the_model_service():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True, healthy=False))
  assert controller.output_v_target == 5.0


def test_disabled_setting_is_transparent():
  controller = ceiling(turn_slowdown_enabled=False)
  update(controller, FakeSM(left_blinker=True))
  assert controller.output_v_target == V_CRUISE_UNSET


def test_inactive_longitudinal_control_is_transparent():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True), long_active=False)
  assert controller.output_v_target == V_CRUISE_UNSET


def test_release_is_limited_to_one_mps2():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True))
  assert controller.output_v_target == 5.0

  previous = controller.output_v_target
  for _ in range(300):
    update(controller, FakeSM())
    if controller.output_v_target == V_CRUISE_UNSET:
      break
    assert controller.output_v_target - previous <= RELEASE_ACCEL_MPS2 * DT + 1e-9
    previous = controller.output_v_target

  assert controller.output_v_target == V_CRUISE_UNSET
  assert controller.is_releasing is False


def test_forgotten_lamp_releases_after_the_wheel_returns_to_centre():
  controller = ceiling()
  update(controller, FakeSM(left_blinker=True, steering=30.0))
  assert controller.output_v_target == 5.0

  for _ in range(14):
    update(controller, FakeSM(left_blinker=True, steering=0.0))
    assert controller.is_active
  assert controller.output_v_target == 5.0

  update(controller, FakeSM(left_blinker=True, steering=0.0))
  assert not controller.is_active
  assert controller.is_releasing
  assert controller.output_v_target > 5.0


def test_turn_cap_follows_the_configured_speed():
  controller = ceiling(turn_speed_kph=12)
  update(controller, FakeSM(right_blinker=True))
  assert controller.output_v_target == 12 / 3.6
