from types import SimpleNamespace

from openpilot.cereal import log, custom
from openpilot.common.params import Params
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase

from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LaneChangeDirection
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import (
  LaneTurnController,
  LANE_CHANGE_SPEED_MIN,
  MANUAL_TURN_SPEED_MAX,
  MANUAL_TURN_SPEED_MARGIN,
)
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode


TurnDirection = custom.ModelDataV2SP.TurnDirection


class TestLaneTurnDesire(OpenpilotTestCase):
  @parameterized.expand([
    (True, False, 5, False, False, TurnDirection.turnLeft),
    (False, True, 6, False, False, TurnDirection.turnRight),
    (True, False, 9, False, False, TurnDirection.none),
    (True, False, 7, True, False, TurnDirection.none),
    (False, True, 6, False, True, TurnDirection.none),
    (False, False, 5, False, False, TurnDirection.none),
    (True, True, 5, False, False, TurnDirection.none),
  ])
  def test_lane_turn_desire_conditions(self, left_blinker, right_blinker, v_ego, blindspot_left, blindspot_right, expected):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    controller.update_lane_turn(blindspot_left, blindspot_right, left_blinker, right_blinker, v_ego)
    assert controller.get_turn_direction() == expected

  def test_lane_turn_desire_disabled(self):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = False
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    controller.update_lane_turn(False, False, True, False, 7)
    assert controller.get_turn_direction() == TurnDirection.none

  def test_lane_turn_overrides_lane_change(self):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    # left turn desire
    controller.update_lane_turn(False, False, True, False, 5)
    assert controller.get_turn_direction() == TurnDirection.turnLeft
    # right turn desire
    controller.update_lane_turn(False, False, False, True, 6)
    assert controller.get_turn_direction() == TurnDirection.turnRight
    # no turn
    controller.update_lane_turn(False, False, False, False, 7)
    assert controller.get_turn_direction() == TurnDirection.none

  @parameterized.expand([
    (8.93, TurnDirection.turnLeft),  # just below threshold
    (8.96, TurnDirection.none),  # above threshold
    (8.95, TurnDirection.none),  # just above threshold
  ])
  def test_lane_turn_desire_speed_boundary(self, v_ego, expected):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    controller.turn_direction = TurnDirection.none
    controller.update_lane_turn(False, True, True, False, v_ego)
    assert controller.get_turn_direction() == expected

  def test_manual_turn_uses_a_higher_speed_limit(self):
    dh = DesireHelper()
    controller = LaneTurnController(dh)
    controller.enabled = True
    controller.lane_turn_value = LANE_CHANGE_SPEED_MIN
    assert MANUAL_TURN_SPEED_MAX == LANE_CHANGE_SPEED_MIN + MANUAL_TURN_SPEED_MARGIN

    controller.update_lane_turn(False, False, True, False, MANUAL_TURN_SPEED_MAX - 0.01)
    assert controller.get_turn_direction() == TurnDirection.none

    controller.update_lane_turn(False, False, True, False, MANUAL_TURN_SPEED_MAX - 0.01, manual_turn=True)
    assert controller.get_turn_direction() == TurnDirection.turnLeft


class DummyCarState:
  def __init__(self, vEgo=0, leftBlinker=False, rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
               steeringPressed=False, steeringTorque=0, brakePressed=False, steeringAngleDeg=0.0):
    self.vEgo = vEgo
    self.leftBlinker = leftBlinker
    self.rightBlinker = rightBlinker
    self.leftBlindspot = leftBlindspot
    self.rightBlindspot = rightBlindspot
    self.steeringPressed = steeringPressed
    self.steeringTorque = steeringTorque
    self.brakePressed = brakePressed
    self.steeringAngleDeg = steeringAngleDeg


def turn_only_intent(direction):
  return SimpleNamespace(valid=True, signalRequested=True, direction=direction, targetLaneIndex=-1,
                         spLaneChangeReady=False)


def set_lane_turn_params():
  params = Params()
  params.put("LaneTurnDesire", True)
  params.put("LaneTurnValue", 20.0)


class TestDesireHelperIntegration(OpenpilotTestCase):
  @parameterized.expand([
    # A matching navigation turn request enables the lane turn desire.
    (DummyCarState(vEgo=5, leftBlinker=True, rightBlinker=False, leftBlindspot=False, rightBlindspot=False), True, 1.0,
     turn_only_intent("left"), log.Desire.turnLeft),
    (DummyCarState(vEgo=7, leftBlinker=False, rightBlinker=True, leftBlindspot=False, rightBlindspot=False), True, 1.0,
     turn_only_intent("right"), log.Desire.turnRight),
    # Lane change desire only (no turn desires)
    (DummyCarState(vEgo=LANE_CHANGE_SPEED_MIN + 1.0, leftBlinker=True, rightBlinker=False,
                   leftBlindspot=False, rightBlindspot=False,
                   steeringPressed=True, steeringTorque=1), True, 1.0, None, log.Desire.laneChangeLeft),
    (DummyCarState(vEgo=LANE_CHANGE_SPEED_MIN + 1.0, leftBlinker=False, rightBlinker=True,
                   leftBlindspot=False, rightBlindspot=False,
                   steeringPressed=True, steeringTorque=-1), True, 1.0, None, log.Desire.laneChangeRight),
    # No desire (inactive)
    (DummyCarState(vEgo=9, leftBlinker=False, rightBlinker=False), False, 1.0, None, log.Desire.none),
    (DummyCarState(vEgo=4, leftBlinker=False, rightBlinker=False), True, 1.0, None, log.Desire.none),  # No blinkers? no desire!
  ], names=["carstate", "lateral_active", "lane_change_prob", "nav_lane_intent", "expected_desire"])
  def test_desire_helper_integration(self, carstate, lateral_active, lane_change_prob, nav_lane_intent,
                                     expected_desire, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    for _ in range(10):
      dh.update(carstate, lateral_active, lane_change_prob,
                left_edge_detected=False, right_edge_detected=False, nav_lane_intent=nav_lane_intent)
    assert dh.desire == expected_desire

  def test_driver_lamp_without_navigation_keeps_lane_keeping(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=5, leftBlinker=True)
    for _ in range(10):
      dh.update(carstate, True, 1.0, left_edge_detected=False, right_edge_detected=False)
    assert dh.lane_turn_direction == TurnDirection.none
    assert dh.desire == log.Desire.none

  def test_lane_change_does_not_start_at_the_speed_boundary(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    carstate = DummyCarState(vEgo=LANE_CHANGE_SPEED_MIN + 0.01, leftBlinker=True)

    for _ in range(10):
      dh.update(carstate, True, 1.0)

    assert dh.lane_change_state == LaneChangeState.off
    assert dh.desire == log.Desire.none

  def test_turned_wheel_cancels_an_intersection_lane_change(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    carstate = DummyCarState(vEgo=LANE_CHANGE_SPEED_MIN + 1.0, leftBlinker=True)

    for _ in range(10):
      dh.update(carstate, True, 1.0)
    assert dh.lane_change_state != LaneChangeState.off

    carstate.steeringAngleDeg = 25.0
    dh.update(carstate, True, 1.0)
    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_change_direction == LaneChangeDirection.none
    assert dh.desire == log.Desire.none

  def test_manual_left_turn_uses_steering_intent_without_navigation(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    carstate = DummyCarState(vEgo=8.5, leftBlinker=True, steeringAngleDeg=5.0)

    for _ in range(10):
      dh.update(carstate, True, 1.0)

    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_turn_direction == TurnDirection.turnLeft
    assert dh.desire == log.Desire.turnLeft

  def test_manual_right_turn_uses_steering_intent_without_navigation(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGELESS
    carstate = DummyCarState(vEgo=8.5, rightBlinker=True, steeringAngleDeg=-5.0)

    for _ in range(10):
      dh.update(carstate, True, 1.0)

    assert dh.lane_change_state == LaneChangeState.off
    assert dh.lane_turn_direction == TurnDirection.turnRight
    assert dh.desire == log.Desire.turnRight

  def test_manual_turn_requires_steering_in_the_signaled_direction(self, set_lane_turn_params):
    dh = DesireHelper()
    carstate = DummyCarState(vEgo=8.5, leftBlinker=True, steeringAngleDeg=-5.0)

    for _ in range(10):
      dh.update(carstate, True, 1.0)

    assert dh.lane_turn_direction == TurnDirection.none
    assert dh.desire == log.Desire.none

  def test_manual_turn_requires_steering_and_lateral_control(self, set_lane_turn_params):
    dh = DesireHelper()
    carstate = DummyCarState(vEgo=8.5, leftBlinker=True, steeringAngleDeg=4.9)
    for _ in range(10):
      dh.update(carstate, True, 1.0)
    assert dh.lane_turn_direction == TurnDirection.none

    carstate.steeringAngleDeg = 5.0
    for _ in range(10):
      dh.update(carstate, False, 1.0)
    assert dh.lane_turn_direction == TurnDirection.none
    assert dh.desire == log.Desire.none

  def test_manual_turn_holds_until_steering_returns_inside_hysteresis(self, set_lane_turn_params):
    dh = DesireHelper()
    carstate = DummyCarState(vEgo=8.5, leftBlinker=True, steeringAngleDeg=5.0)
    dh.update(carstate, True, 1.0)
    assert dh.lane_turn_direction == TurnDirection.turnLeft

    carstate.steeringAngleDeg = 3.0
    dh.update(carstate, True, 1.0)
    assert dh.lane_turn_direction == TurnDirection.turnLeft

    carstate.steeringAngleDeg = 1.0
    dh.update(carstate, True, 1.0)
    assert dh.lane_turn_direction == TurnDirection.none
    assert dh.desire == log.Desire.none

  def test_edge_blocks_lane_change(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=15, leftBlinker=True, steeringPressed=True, steeringTorque=1)
    for _ in range(10):
      dh.update(carstate, True, 1.0, left_edge_detected=True, right_edge_detected=False)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.desire == log.Desire.none

  def test_solid_line_blocks_signaled_lane_change_until_the_line_clears(self, set_lane_turn_params):
    dh = DesireHelper()
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    carstate = DummyCarState(vEgo=15, leftBlinker=True, steeringPressed=True, steeringTorque=1)
    for _ in range(10):
      dh.update(
        carstate, True, 1.0,
        left_line_blocked=True, right_line_blocked=False,
      )

    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.lane_change_direction == LaneChangeDirection.left
    assert dh.desire == log.Desire.none

    dh.update(
      carstate, True, 1.0,
      left_line_blocked=False, right_line_blocked=False,
    )
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft
