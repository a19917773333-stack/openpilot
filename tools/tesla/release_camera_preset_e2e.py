#!/usr/bin/env python3
"""Execute the real model SConscript's device camera selection without building."""
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch

from openpilot.common.transformations.camera import DEVICE_CAMERAS

root = Path(__file__).resolve().parents[2]
source = (root/'openpilot/selfdrive/modeld/SConscript').read_text()
selection = source[source.index("if arch == 'comma_arm64':"):source.index('\ntg_devices =')]
rows = []
for device, profile, expected in [('mici', 'standard', (1344, 760)), ('tici', 'c3', (1928, 1208)),
                                  ('tizi', 'standard', (1928, 1208))]:
  hardware = ModuleType('openpilot.common.hardware')
  hardware.HARDWARE = SimpleNamespace(get_device_type=lambda: device)
  context = {'arch': 'comma_arm64', 'os': os, 'DEVICE_CAMERAS': DEVICE_CAMERAS}
  with patch.dict(sys.modules, {'openpilot.common.hardware': hardware}), \
       patch.dict(os.environ, {'SUNNYPILOT_HARDWARE_PROFILE': profile}):
    exec(compile(selection, 'model-camera-selection', 'exec'), context)
  assert context['camera_configs'] == [expected], context['camera_configs']
  rows.append({'device': device, 'profile': profile, 'camera_configs': context['camera_configs']})
print(json.dumps({'pass': True, 'scope': 'actual SCons device camera selection; no model/native/device build', 'cases': rows}, indent=2))
