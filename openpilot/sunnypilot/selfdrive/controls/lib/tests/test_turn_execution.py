from types import SimpleNamespace

import pytest

from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.sunnypilot.selfdrive.controls.lib.turn_entry import TurnEntryGate, TurnManeuver


def car_state(v_ego=7.5):
  return SimpleNamespace(vEgo=v_ego, aEgo=0.0, steeringAngleDeg=0.0,
                         leftBlinker=True, rightBlinker=False)


def cp_model():
  def line(y):
    return SimpleNamespace(y=[y] * len(ModelConstants.T_IDXS))

  edges = [SimpleNamespace(y=[side * (3.8 + 4.0 * max(0.0, min(1.0, t - 1.0))) for t in ModelConstants.T_IDXS])
           for side in (-1, 1)]
  return SimpleNamespace(
    laneLines=[line(-5.3), line(-1.8), line(1.8), line(5.3)],
    roadEdges=edges,
    roadEdgeStds=[0.9, 0.9],
    laneLineProbs=[0.1, 0.9, 0.9, 0.1],
    meta=SimpleNamespace(desireState=[1.0, 0.0, 0.0]),
  )


def topology(stamp):
  return SimpleNamespace(
    validForControl=True, leftNeighborExists=False, rightNeighborExists=False,
    leftEgoSideMarking="unknown", rightEgoSideMarking="unknown",
    leftEvidenceValid=False, rightEvidenceValid=False,
    publishMonoTime=stamp, modelMonoTime=stamp, imageMonoTime=stamp,
  )


def navigation(stamp, *, direction="left"):
  intent = SimpleNamespace(valid=True, signalRequested=True, direction=direction, targetLaneIndex=-1,
                           sessionId="session", routeRevision=1, maneuverEventId=2)
  state = SimpleNamespace(valid=True, stale=False, gpsWeak=False, routeActive=True, routeMatched=True,
                          publishMonoTime=stamp, sessionId="session", routeRevision=1,
                          maneuverEventId=2, maneuver="turn" + direction.title())
  return intent, state


def gate_tick(gate, step, *, neighbors, nav=False):
  stamp = 1_000_000_000 + step * 50_000_000
  nav_intent, nav_state = navigation(stamp) if nav else (None, None)
  return gate.update(
    cp_model(), topology(stamp), healthy=True, now_ns=stamp, model_stamp_ns=stamp,
    neighbors=neighbors, carstate=car_state(), nav_intent=nav_intent, nav_state=nav_state,
  )


@pytest.mark.parametrize("nav", [False, True])
def test_turn_entry_needs_the_same_clear_neighbor_with_or_without_navigation(nav):
  gate = TurnEntryGate()
  for step in range(13):
    assert gate_tick(gate, step, neighbors=(None, None), nav=nav) == (False, False)
  # Clearing an unknown neighbor starts a new classification history; the
  # navigation request cannot reuse evidence collected while the side was unknown.
  for step in range(13, 25):
    assert gate_tick(gate, step, neighbors=(False, None), nav=nav) == (False, False)
  assert gate_tick(gate, 25, neighbors=(False, None), nav=nav) == (True, False)


def test_turn_maneuver_starts_without_driver_steering_and_finishes_with_lamp():
  maneuver = TurnManeuver()
  common = {"eligible": True, "entry_allowed": True, "keep_allowed": True, "hard_blocked": False}
  assert not maneuver.update(None, **common)
  assert maneuver.update("left", **common)
  assert maneuver.state == "active"
  assert not maneuver.update(None, **common)
  assert maneuver.state == "waiting"


def test_turn_maneuver_hard_block_and_completion_consume_the_lamp_session():
  for stop in ("hard_blocked", "completed"):
    maneuver = TurnManeuver()
    common = {"eligible": True, "entry_allowed": True, "keep_allowed": True,
              "hard_blocked": False, "completed": False}
    assert maneuver.update("right", **common)
    assert not maneuver.update("right", **(common | {stop: True}))
    assert maneuver.state == "finished"
