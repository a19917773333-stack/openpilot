from openpilot.cereal import custom
from openpilot.common.params import UnknownKeyName
from openpilot.common.parameterized import parameterized
from openpilot.common.test import OpenpilotTestCase

from openpilot.selfdrive.controls.lib.desire_helper import (
  LANE_TURN_CURVATURE_SCALE_DEFAULT,
  LANE_TURN_CURVATURE_SCALE_MAX,
  LANE_TURN_CURVATURE_SCALE_MIN,
  get_lane_turn_curvature_scale,
  scale_turn_curvature,
)


TurnDirection = custom.ModelDataV2SP.TurnDirection
PARAM_NAME = "LaneTurnCurvatureScale"


class FakeParams:
  def __init__(self, value=None):
    self.value = value

  def get(self, key):
    assert key == PARAM_NAME
    return self.value


class UnknownParams:
  def get(self, key):
    raise UnknownKeyName(key)


class TestTurnCurvatureScale(OpenpilotTestCase):
  def test_missing_param_uses_default(self):
    self.assertAlmostEqual(get_lane_turn_curvature_scale(FakeParams()), LANE_TURN_CURVATURE_SCALE_DEFAULT)

  def test_unregistered_param_uses_default_instead_of_crashing(self):
    self.assertAlmostEqual(get_lane_turn_curvature_scale(UnknownParams()), LANE_TURN_CURVATURE_SCALE_DEFAULT)

  @parameterized.expand([(b"0.70", 0.70), ("0.80", 0.80), (0.75, 0.75)])
  def test_accepts_bytes_str_and_float(self, raw, expected):
    self.assertAlmostEqual(get_lane_turn_curvature_scale(FakeParams(raw)), expected)

  @parameterized.expand([("", LANE_TURN_CURVATURE_SCALE_DEFAULT), ("bad", LANE_TURN_CURVATURE_SCALE_DEFAULT),
                         ("0.10", LANE_TURN_CURVATURE_SCALE_MIN), ("2.0", LANE_TURN_CURVATURE_SCALE_MAX)])
  def test_invalid_and_out_of_range_values_are_clamped(self, raw, expected):
    self.assertAlmostEqual(get_lane_turn_curvature_scale(FakeParams(raw)), expected)

  @parameterized.expand([(TurnDirection.none, 1.0), (TurnDirection.turnLeft, 0.75),
                         (TurnDirection.turnRight, 0.75)])
  def test_only_active_turns_are_scaled(self, direction, expected_scale):
    self.assertAlmostEqual(scale_turn_curvature(100.0, direction, 0.75), 100.0 * expected_scale)

  def test_scale_is_clamped_for_turns(self):
    self.assertAlmostEqual(scale_turn_curvature(100.0, TurnDirection.turnLeft, 0.10),
                           100.0 * LANE_TURN_CURVATURE_SCALE_MIN)
    self.assertAlmostEqual(scale_turn_curvature(100.0, TurnDirection.turnRight, 2.0),
                           100.0 * LANE_TURN_CURVATURE_SCALE_MAX)
