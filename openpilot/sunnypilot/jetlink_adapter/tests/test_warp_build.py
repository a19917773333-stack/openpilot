"""
Copyright (c) 2026-, Zeph Leggett.

This file is part of zoompilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

The warp jetlink runs is modeld's own build product.

Jetlink 0.8.7 asks the adapter where modeld's driving warp is and runs that
pickle itself. The adapter must not compile a second warp, and modeld must
keep building one for every camera it installs on.
"""
import unittest
from pathlib import Path

from openpilot.common.basedir import BASEDIR
from openpilot.common.test import OpenpilotTestCase
from openpilot.selfdrive.modeld.helpers import MODELS_DIR
from openpilot.sunnypilot import jetlink_adapter


class TestWarpBuild(OpenpilotTestCase):
  def test_the_adapter_points_at_modelds_driving_warp(self):
    geometry = (1928, 1208, 512, 256)
    expected = MODELS_DIR / 'driving_warp_1928x1208_tinygrad.pkl'
    self.assertEqual(jetlink_adapter.warp_path(*geometry), expected)
    self.assertEqual(jetlink_adapter.Adapter().warp_path(*geometry), expected)

  def test_modeld_still_builds_the_driving_warp(self):
    sconscript = Path(BASEDIR) / 'openpilot' / 'selfdrive' / 'modeld' / 'SConscript'
    self.assertIn('driving_warp_{cam_w}x{cam_h}_tinygrad.pkl', sconscript.read_text())

  def test_the_adapter_has_no_separate_warp_build(self):
    sconscript = Path(BASEDIR) / 'openpilot' / 'sunnypilot' / 'SConscript'
    self.assertNotIn('jetlink_adapter/SConscript', sconscript.read_text())
    self.assertFalse((Path(BASEDIR) / 'openpilot' / 'sunnypilot' / 'jetlink_adapter' / 'SConscript').exists())


if __name__ == '__main__':
  unittest.main()
