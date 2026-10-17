#!/usr/bin/env python3
"""Replay real daemon entrypoints with hardware, IPC and scheduling boundaries replaced."""
import argparse
import hashlib
import json
from pathlib import Path
import signal
from types import SimpleNamespace
from unittest.mock import patch

from openpilot.selfdrive.pandad import pandad
from openpilot.selfdrive.locationd import torqued

ROOT = Path(__file__).resolve().parents[2]


def panda_case(stage):
  events = []
  handler = None
  scans = 0

  def register(_, callback):
    nonlocal handler
    handler = callback

  def interrupt():
    handler(signal.SIGINT, None)

  def enumerate_pandas():
    nonlocal scans
    scans += 1
    if scans == 1:
      return []
    if stage == 'enumeration':
      interrupt()
    return ['internal-serial']

  def flash(serial):
    events.append(['flash', serial])
    if stage == 'flash':
      interrupt()

  class Child:
    def __init__(self, args, **kwargs):
      events.append(['spawn', args])
      if stage == 'spawn':
        interrupt()

    def send_signal(self, value):
      events.append(['signal', int(value)])

    def wait(self):
      if stage in ('running', 'reset'):
        interrupt()

  with patch.object(pandad.signal, 'signal', register), \
       patch.object(pandad.Panda, 'list', enumerate_pandas), \
       patch.object(pandad.Panda, 'wait_for_panda', side_effect=lambda _serial, timeout: events.append(['enumeration_wait', timeout])), \
       patch.object(pandad.PandaDFU, 'list', return_value=[]), \
       patch.object(pandad, 'PandaStartup', return_value=SimpleNamespace(prepare=lambda *_: pandad.PandaStartupResult.RESET if stage == 'reset' else pandad.PandaStartupResult.APP_READY)), \
       patch.object(pandad, 'check_panda_support', side_effect=lambda serials: serials), \
       patch.object(pandad, 'flash_rivian_long'), \
       patch.object(pandad, 'flash_panda', flash), \
       patch.object(pandad.subprocess, 'Popen', Child):
    pandad.main()
  assert any(e[0] == 'spawn' for e in events) == (stage in ('spawn', 'running', 'reset')), events
  assert any(e[0] == 'flash' for e in events) == (stage != 'enumeration'), events
  if stage in ('spawn', 'running', 'reset'):
    assert ['spawn', ['./pandad', 'internal-serial']] in events, events
    assert ['signal', int(signal.SIGINT)] in events, events
  assert (['enumeration_wait', 2] in events) == (stage == 'reset'), events
  return events


def torque_case(write_failure):
  phase = 'normal'
  writes = []
  frames = 0

  class ReplayDone(Exception):
    pass

  def realtime(*_):
    nonlocal phase
    phase = 'realtime'

  def normal():
    nonlocal phase
    phase = 'normal'

  class Samples:
    updated = {}
    frame = 0

    def update(self):
      nonlocal frames
      self.frame += 1
      frames = self.frame
      if self.frame > 1201:
        raise ReplayDone()

    def all_checks(self):
      return False

  class Storage:
    def get(self, *_args, **_kwargs):
      return b''

    def put(self, key, value):
      assert phase == 'normal', 'cache write still runs at realtime priority'
      writes.append({'frame': frames, 'key': key, 'value': value.decode()})
      if write_failure:
        raise OSError('injected cache write failure')

  def message(**kwargs):
    if kwargs.get('with_points'):
      assert phase == 'normal', 'point serialization still runs at realtime priority'
    return SimpleNamespace(to_bytes=lambda: b'learning-points')

  error = None
  with patch.object(torqued, 'config_realtime_process', realtime), \
       patch.object(torqued, 'drop_realtime', normal, create=True), \
       patch.object(torqued, 'Params', Storage), \
       patch.object(torqued.messaging, 'SubMaster', return_value=Samples()), \
       patch.object(torqued.messaging, 'PubMaster', return_value=SimpleNamespace(send=lambda *_: None)), \
       patch.object(torqued.messaging, 'log_from_bytes', return_value=None), \
       patch.object(torqued, 'TorqueEstimator', return_value=SimpleNamespace(get_msg=message)), \
       patch.object(torqued.TorqueEstimatorExt, 'update_use_params'):
    try:
      torqued.main()
    except ReplayDone:
      pass
    except OSError as exc:
      error = str(exc)
  assert phase == 'realtime', 'scheduler was not restored'
  assert [w['frame'] for w in writes] == [1200], writes
  assert bool(error) == write_failure, error
  return {'writes': writes, 'failure_propagated': bool(error), 'scheduler_restored': phase}


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  rows = []
  for name, run in [(f'panda-{stage}', lambda s=stage: panda_case(s)) for stage in ('enumeration', 'flash', 'spawn', 'running', 'reset')] + \
                   [(f'torque-write-failure-{failure}', lambda f=failure: torque_case(f)) for failure in (False, True)]:
    try:
      rows.append({'case': name, 'pass': True, 'result': run()})
    except Exception as exc:
      rows.append({'case': name, 'pass': False, 'error': repr(exc)})
  report = {'scope': 'actual daemon entrypoint orchestration, synthetic IPC/hardware/scheduler boundaries; no devices',
            'sources': {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in
                        ('openpilot/selfdrive/pandad/pandad.py', 'openpilot/selfdrive/locationd/torqued.py')},
            'cases': rows, 'pass': all(row['pass'] for row in rows)}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(report, indent=2)+'\n')
  print(json.dumps(report, indent=2))
  return 0 if report['pass'] else 1


if __name__ == '__main__':
  raise SystemExit(main())
