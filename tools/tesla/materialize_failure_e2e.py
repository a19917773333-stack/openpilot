#!/usr/bin/env python3
"""Replay checkout failures at a conditional shell boundary where errexit is suppressed."""
import json
import os
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[2]
source = (root/'tools/release/device_release.sh').read_text()
function = re.search(r'^checkout_repo\(\) \{\n.*?^\}', source, re.M | re.S).group()
rows = []
for failure in ('none', 'fetch', 'checkout', 'clean'):
  harness = r'''
set -Eeuo pipefail
die() { echo "$*" >&2; exit 1; }
ensure_repo() { :; }
git_fetch_retry() { [ "$FAILURE" != fetch ]; }
git() { [ "$FAILURE" != "$3" ]; }
''' + function + '\ncheckout_repo fixture local sha || exit 77\n'
  result = subprocess.run(['bash', '-c', harness], env={**os.environ, 'FAILURE': failure}, text=True, capture_output=True)
  assert (result.returncode == 0) == (failure == 'none'), (failure, result.stdout, result.stderr)
  rows.append({'case': failure, 'pass': True, 'exit': result.returncode})
print(json.dumps({'pass': True, 'scope': 'actual checkout function with failing Git boundary; no device/network/filesystem mutation', 'cases': rows}, indent=2))
