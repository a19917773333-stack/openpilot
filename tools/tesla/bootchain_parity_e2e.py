#!/usr/bin/env python3
"""Compare exact SP boot entries and excluded updater/setup files; never access a device."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

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
    for name in ('abl', 'boot'):
      expected = next(p for p in source if p['name'] == name)
      actual = next(p for p in current if p['name'] == name)
      assert actual == expected, (profile, name, actual, expected)
      rows.append({'profile': profile, 'partition': name, 'pass': True, 'entry': actual})
  excluded = {}
  for path in ['openpilot/common/hardware/comma/updater', 'openpilot/system/ui/updater.py',
               'openpilot/system/ui/mici_setup.py', 'openpilot/system/ui/tici_setup.py',
               'openpilot/common/hardware/comma/agnos.json', 'openpilot/common/hardware/comma/agnos-c3.json',
               'openpilot/common/hardware/comma/agnos-c3xl.json', 'launch_env.sh']:
    original = subprocess.check_output(['git', '-C', str(ROOT), 'show', f'e0814246c918439f8d05d4518efaa649753e3f3b:{path}'])
    actual = (ROOT/path).read_bytes()
    assert actual == original, path
    excluded[path] = hashlib.sha256(actual).hexdigest()
  result = {'pass': True, 'scope': 'source manifest equality and excluded-file byte equality; no partition readback or device access',
            'source_repo': str(args.source_repo), 'source_commit': source_sha, 'boot_chain': rows,
            'excluded_unchanged_sha256': excluded}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(result, indent=2)+'\n')
  print(json.dumps({'pass': True, 'source': source_sha, 'partition_cases': len(rows), 'unchanged_files': len(excluded)}))


if __name__ == '__main__':
  main()
