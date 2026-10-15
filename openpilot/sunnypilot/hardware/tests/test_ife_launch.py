import json
import os
from pathlib import Path
import subprocess
import sys

import pytest


REPO_ROOT = Path(__file__).parents[4]


@pytest.mark.parametrize('env_profile,file_profile,model,override,scaled', [
  (None, 'c3xl', 'comma tici', None, True),
  (None, 'c3xl', 'comma tici', '1344x760', True),
  (None, 'c3xl', 'comma tici', '1928x1208', False),
  (None, 'c3xl', 'comma tici', '', False),
  (None, None, 'comma tici', None, True),
  (None, None, 'comma tizi', None, False),
  (None, None, 'comma mici', None, False),
  (None, None, 'unknown', None, False),
  (None, 'standard', 'comma tici', None, False),
  (None, 'c3xl', 'comma tizi', None, True),
  ('c3xl', None, 'unknown', None, True),
  ('standard', 'c3xl', 'comma tici', None, False),
])
def test_launch_exports_matching_road_geometry(tmp_path, env_profile, file_profile, model, override, scaled):
  profile_path = tmp_path / 'hardware_profile'
  if file_profile is not None:
    profile_path.write_text(file_profile + '\n')
  model_path = tmp_path / 'model'
  model_path.write_bytes(model.encode() + b'\x00')
  env = os.environ.copy()
  for key in ('SUNNYPILOT_HARDWARE_PROFILE', 'C3XL_IFE_ROAD_SIZE', 'AGNOS_VERSION', 'AGNOS_MANIFEST_FILE'):
    env.pop(key, None)
  env.update(SUNNYPILOT_HARDWARE_PROFILE_FILE=str(profile_path), SUNNYPILOT_HARDWARE_MODEL_FILE=str(model_path),
             PYTHONPATH=str(REPO_ROOT))
  if env_profile is not None:
    env['SUNNYPILOT_HARDWARE_PROFILE'] = env_profile
  if override is not None:
    env['C3XL_IFE_ROAD_SIZE'] = override
  probe = '''
import json, os
from openpilot.common.transformations import camera
config = camera.DEVICE_CAMERAS[('tici', 'ox03c10')]
print(json.dumps({
  'source': camera.__file__,
  'resize': os.getenv('C3XL_IFE_ROAD_SIZE'),
  'sizes': [cam.size for _, cam in config.all_cams()],
  'intrinsics': [cam.intrinsics.tolist() for _, cam in config.all_cams()],
  'ar0231': camera.DEVICE_CAMERAS[('tici', 'ar0231')].wide_road.size,
  'os04c10': camera.DEVICE_CAMERAS[('mici', 'os04c10')].wide_road.size,
}))
'''
  result = json.loads(subprocess.check_output(
    ['bash', '-c', 'source launch_env.sh; source launch_env.sh; exec "$@"', 'ife-launch', sys.executable, '-c', probe],
    cwd=REPO_ROOT, env=env, text=True))
  assert Path(result['source']) == REPO_ROOT / 'openpilot/common/transformations/camera.py'
  assert result['resize'] == ('1344x760' if scaled else override)
  road_size = [1344, 760] if scaled else [1928, 1208]
  assert result['sizes'] == [road_size, [1928, 1208], road_size]
  sx, sy = road_size[0] / 1928, road_size[1] / 1208
  for index, focal_length in ((0, 2648), (1, 567), (2, 567)):
    x, y = (1, 1) if index == 1 else (sx, sy)
    expected = [[focal_length*x, 0, 964*x], [0, focal_length*y, 604*y], [0, 0, 1]]
    for actual_row, expected_row in zip(result['intrinsics'][index], expected, strict=True):
      assert actual_row == pytest.approx(expected_row)
  assert result['ar0231'] == [1928, 1208]
  assert result['os04c10'] == [1344, 760]
