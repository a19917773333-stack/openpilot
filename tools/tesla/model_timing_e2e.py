#!/usr/bin/env python3
"""Reply ingress -> real parser/latch/blender/action -> serialized modelV2 replay."""
import argparse
import hashlib
import json
from pathlib import Path
import threading

import numpy as np
from openpilot.cereal import messaging, log
from openpilot.selfdrive.modeld.big_model import (BigReplyLatch, FrameDelayEstimator, SourceBlender,
                                                nanos_since_boot, parse_big_outputs, select_frame)
from openpilot.selfdrive.modeld.modeld import ModelState
from openpilot.selfdrive.modeld.fill_model_msg import PublishState, fill_model_msg


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  parser.add_argument('--historical', type=Path, action='append', default=[])
  args = parser.parse_args()
  model = ModelState.__new__(ModelState)
  model.LAT_SMOOTH_SECONDS, model.LONG_SMOOTH_SECONDS = 0., 0.3
  raw = np.zeros(2066, dtype=np.float32)
  raw[2062:2064] = [1.5, -2.]
  small = parse_big_outputs(raw)
  rows = []
  eof = nanos_since_boot() - 500_000_000
  for name, arrival_ms, enabled, run_count, expected in [
    ('before', 59.9, True, 6, 'big'), ('boundary', 60., True, 6, 'big'),
    ('after', 60.1, True, 6, 'late'), ('disabled', 20., False, 6, 'off'),
    ('warmup', 20., True, 5, 'warmup'), ('disconnected', None, True, 6, 'linkDown'),
    ('zero', 20., True, 6, 'zeroOutput'), ('stale', 20., True, 6, 'timeout')]:
    latch = BigReplyLatch(type('Idle', (), {'update': lambda self, _: threading.Event().wait()})())
    if arrival_ms is not None:
      reply = np.zeros_like(raw) if name == 'zero' else raw
      stamp = eof - 50_000_000 if name == 'stale' else eof
      latch._on_reply(stamp, reply, stamp + int(arrival_ms * 1e6))
    frame = select_frame(small, latch=latch, enabled=enabled, run_count=run_count,
                         timestamp_eof=eof, deadline_after_eof_ms=60., warmup_frames=5,
                         current_time_ns=nanos_since_boot, blender=SourceBlender(),
                         action_for=lambda out, lat, lon: model.get_action_from_model(out, log.ModelDataV2.Action(), lat, lon, 20.),
                         action_t=(.125, .45))
    assert frame.source == expected, (name, frame.source)
    msg = messaging.new_message('modelV2')
    action = log.ModelDataV2.Action(desiredCurvature=frame.desired_curvature,
                                   desiredAcceleration=frame.desired_acceleration, shouldStop=frame.should_stop)
    fill_model_msg(msg, frame.model_output, action, PublishState(), 1, 1, 1, 0., eof, .02, True)
    msg.modelV2.big = frame.big_output is not None
    with log.Event.from_bytes(msg.to_bytes()) as decoded:
      assert decoded.modelV2.big == (name in ('before', 'boundary'))
      assert np.isfinite([decoded.modelV2.action.desiredCurvature, decoded.modelV2.action.desiredAcceleration]).all()
    rows.append({'case': name, 'source': frame.source, 'deadline_ms': frame.deadline_ms,
                 'curvature': frame.desired_curvature, 'acceleration': frame.desired_acceleration})
  estimator = FrameDelayEstimator()
  assert estimator.value == 50.
  assert estimator.update(1.) == 10.
  for _ in range(100):
    estimator.update(200.)
  assert estimator.value == 100.
  for _ in range(100):
    estimator.update(34.)
  assert estimator.value == 34.
  history = []
  for path in args.historical:
    data = json.loads(path.read_text())
    samples = [row['latency_ms'] for row in data['rows'] if row.get('source') == 'big' and row['latency_ms'] > 0]
    assert samples and max(samples) < 60., (path, max(samples))
    history.append({'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                    'samples': len(samples), 'max_ms': max(samples), 'within_60_ms': len(samples),
                    'scope': 'historical one-second sampled accepted replies; not all frames or current-device proof'})
  result = {'pass': True, 'scope': 'host reply-to-modelV2 pipeline with synthetic ingress; no camera/encoder/phone execution',
            'cases': rows, 'historical': history, 'frame_delay_ms': estimator.value}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2)+'\n')
  print(json.dumps(result, indent=2))


if __name__ == '__main__':
  main()
