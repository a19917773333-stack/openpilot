#!/usr/bin/env python3
"""Compare Python/native calibrated warps plus actual padded NV12 and BGM1 headers."""
import argparse
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace

import numpy as np
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.modeld.camera_offset_helper import CameraOffsetHelper

ROOT = Path(__file__).resolve().parents[2]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  args.output.parent.mkdir(parents=True, exist_ok=True)
  binary = args.output.parent/'bigmodel_geometry_e2e'
  native = ROOT/'openpilot/selfdrive/bigmodeld'
  subprocess.run(['clang++', '-std=c++17', '-O1', '-I'+str(native),
                  str(ROOT/'tools/tesla/bigmodel_geometry_e2e.cc'),
                  *[str(native/f) for f in ('frame_meta.cpp', 'frame_codec.cpp', 'warp_pack.cpp')],
                  '-o', str(binary)], check=True)
  reports = []
  for offset in (0., .2, -.2):
    report = json.loads(subprocess.check_output([str(binary), str(offset)], text=True))
    for case in report['cases']:
      w, h, wide = case['width'], case['height'], case['wide']
      ox = case['sensor'] == 'ar_ox'
      f = (567. if wide else 2648.) if ox else (425.25 if wide else 1141.5)
      k = np.array([[f*w/1928 if ox else f, 0, w/2],
                    [0, f*h/1208 if ox else f, h/2], [0, 0, 1]], dtype=np.float32)
      rpy = np.array([.01, -.02, .005], dtype=np.float32)
      base = get_warp_matrix(rpy, k, wide).astype(np.float32)
      helper = CameraOffsetHelper()
      helper.set_offset(float(np.float32(offset)))
      for _ in range(12):
        expected, _ = helper.update(base, base, k, k, SimpleNamespace(height=[float(np.float32(1.22))], rpyCalib=rpy))
      got = np.array(case['warp'], dtype=np.float32).reshape(3, 3)
      np.testing.assert_allclose(got, expected, atol=2e-4, rtol=1e-6)
      case['python_max_abs_error'] = float(np.max(np.abs(got-expected)))
    reports.append(report)
  result = {'pass': True, 'scope': 'Python and compiled native geometry, actual LUT/padded NV12, BGM1 header roundtrip; no hardware',
            'cases': sum(len(r['cases']) for r in reports), 'reports': reports}
  args.output.write_text(json.dumps(result, indent=2)+'\n')
  print(json.dumps({'pass': True, 'cases': result['cases']}))


if __name__ == '__main__':
  main()
