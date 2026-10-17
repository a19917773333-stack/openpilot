#!/usr/bin/env python3
"""Run native-build shell environment with a caller-local dependency, no build/device IO."""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root/'tools/release/device_release.sh').read_text()
assignment = re.search(r'^\s*PYTHONPATH=.*', source, re.M).group().strip().split(' scons ', 1)[0]
results = []
with tempfile.TemporaryDirectory(prefix='native path ') as tmp:
  tmp = Path(tmp); src = tmp/'source'; deps = tmp/'caller deps'
  (src/'openpilot').mkdir(parents=True); deps.mkdir()
  (deps/'native_optional_dependency.py').write_text('VALUE = 93\n')
  for inherited in (None, '', str(deps)):
    env = os.environ.copy(); env.pop('PYTHONPATH',None)
    env.update(SRC=str(src), EXPECT_EXTERNAL=str(bool(inherited)), EXPECT_DEPS=str(deps))
    if inherited is not None:
      env['PYTHONPATH'] = inherited
    code = '''import os,json
paths=os.environ['PYTHONPATH'].split(':')
assert paths[:2]==[os.environ['SRC'],os.environ['SRC']+'/openpilot']
assert '' not in paths
if os.environ['EXPECT_EXTERNAL']=='True':
 import native_optional_dependency
 assert native_optional_dependency.VALUE==93
 assert paths[2:]==[os.environ['EXPECT_DEPS']]
else:
 assert len(paths)==2
print(json.dumps({'pythonpath':paths,'pass':True}))
'''
    p=subprocess.run(['bash','-c',assignment+' "$1" -c "$2"','native-env-check',sys.executable,code],env=env,capture_output=True,text=True)
    assert p.returncode==0,(inherited,p.stderr)
    results.append({'inherited':inherited,'result':json.loads(p.stdout)})
print(json.dumps({'pass':True,'scope':'actual native shell environment only; no SCons or device execution','cases':results},indent=2))
