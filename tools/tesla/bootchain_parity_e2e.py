#!/usr/bin/env python3
"""Compare SP boot/system entries, launcher selection and excluded updater/setup files."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--source-repo', type=Path, required=True)
  parser.add_argument('--source-ref', required=True)
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  source_sha = subprocess.check_output(['git', '-C', str(args.source_repo), 'rev-parse', args.source_ref], text=True).strip()
  rows = []
  for profile, source_name in [('c3', 'agnos.json'), ('c3xl', 'agnos-c3xl.json')]:
    source = json.loads(subprocess.check_output(['git', '-C', str(args.source_repo), 'show',
                                               f'{source_sha}:openpilot/common/hardware/comma/{source_name}']))
    current = json.loads((ROOT/f'openpilot/common/hardware/comma/agnos-{profile}.json').read_text())
    for name in ('abl', 'boot', 'system'):
      expected = next(p for p in source if p['name'] == name)
      actual = next(p for p in current if p['name'] == name)
      assert actual == expected, (profile, name, actual, expected)
      rows.append({'profile': profile, 'partition': name, 'pass': True, 'entry': actual})
  source_launch = subprocess.check_output(['git', '-C', str(args.source_repo), 'show', f'{source_sha}:launch_env.sh'], text=True)
  source_version = re.search(r'export AGNOS_VERSION="([^"]+)"', source_launch).group(1)
  launches = []
  with tempfile.TemporaryDirectory() as directory:
    model_file, profile_file = Path(directory)/'model', Path(directory)/'profile'
    for model, profile, from_file, expected_manifest, expected_version, skip in [
      ('tici', 'c3', False, 'agnos-c3.json', source_version, '0'),
      ('tici', 'c3xl', False, 'agnos-c3xl.json', source_version, '0'),
      ('tici', 'c3', True, 'agnos-c3.json', source_version, '0'),
      ('tici', 'c3xl', True, 'agnos-c3xl.json', source_version, '0'),
      ('tici', '', True, 'agnos.json', '19.6.27', '1'),
      ('tici', 'standard', False, 'agnos.json', '19.6.27', '1'),
      ('mici', 'standard', False, 'agnos.json', '19.6.27', '0'),
      ('tizi', 'standard', False, 'agnos.json', '19.6.27', '0')]:
      model_file.write_text('comma '+model)
      profile_file.write_text(profile if from_file else '')
      env = {k: v for k, v in os.environ.items() if k not in ('AGNOS_VERSION', 'AGNOS_ACCEPTED_VERSIONS', 'C3XL_IFE_ROAD_SIZE')}
      env.update(SUNNYPILOT_HARDWARE_MODEL_FILE=str(model_file), SUNNYPILOT_HARDWARE_PROFILE_FILE=str(profile_file),
                 SUNNYPILOT_HARDWARE_PROFILE='' if from_file else profile)
      command = 'source ./launch_env.sh; printf "%s\\n" "${AGNOS_MANIFEST_FILE##*/}" "$AGNOS_VERSION" "$AGNOS_ACCEPTED_VERSIONS" "$AGNOS_SKIP_UPDATE"'
      actual = subprocess.check_output(['bash', '-c', command], cwd=ROOT, env=env, text=True).splitlines()
      expected = [expected_manifest, expected_version, expected_version, skip]
      assert actual == expected, (model, profile, from_file, actual, expected)
      launches.append({'device': model, 'profile': profile, 'from_file': from_file, 'result': actual, 'pass': True})
    env.update(AGNOS_VERSION='explicit-version', AGNOS_ACCEPTED_VERSIONS='explicit-version another-version',
               SUNNYPILOT_HARDWARE_PROFILE='c3xl')
    model_file.write_text('comma tici')
    actual = subprocess.check_output(['bash', '-c', command], cwd=ROOT, env=env, text=True).splitlines()
    assert actual == ['agnos-c3xl.json', 'explicit-version', 'explicit-version another-version', '0'], actual
    launches.append({'case': 'explicit caller overrides', 'result': actual, 'pass': True})
    launcher = (ROOT/'launch_chffrplus.sh').read_text()
    reset = next(line.strip() for line in launcher.splitlines() if line.strip().startswith('unset AGNOS_VERSION'))
    env.update(AGNOS_VERSION='19.6.27', AGNOS_ACCEPTED_VERSIONS='19.6.27')
    actual = subprocess.check_output(['bash', '-c', reset+'; '+command], cwd=ROOT, env=env, text=True).splitlines()
    assert actual == ['agnos-c3xl.json', source_version, source_version, '0'], actual
    launches.append({'case': 'overlay restart clears inherited version defaults', 'result': actual, 'pass': True})
  excluded = {}
  for path in ['openpilot/common/hardware/comma/updater', 'openpilot/system/ui/updater.py',
               'openpilot/system/ui/mici_setup.py', 'openpilot/system/ui/tici_setup.py',
               'openpilot/common/hardware/comma/agnos.json']:
    original = subprocess.check_output(['git', '-C', str(ROOT), 'show', f'e0814246c918439f8d05d4518efaa649753e3f3b:{path}'])
    actual = (ROOT/path).read_bytes()
    assert actual == original, path
    excluded[path] = hashlib.sha256(actual).hexdigest()
  result = {'pass': True, 'scope': 'source manifest equality and excluded-file byte equality; no partition readback or device access',
            'source_repo': str(args.source_repo), 'source_commit': source_sha, 'source_version': source_version,
            'boot_chain': rows, 'launches': launches,
            'excluded_unchanged_sha256': excluded}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2)+'\n')
  print(json.dumps({'pass': True, 'source': source_sha, 'partition_cases': len(rows), 'launch_cases': len(launches), 'unchanged_files': len(excluded)}))


if __name__ == '__main__':
  main()
