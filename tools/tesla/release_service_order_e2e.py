#!/usr/bin/env python3
"""Replay actual release stage ordering and EXIT cleanup without systemctl/device IO."""
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

source=(Path(__file__).resolve().parents[2]/'tools/release/device_release.sh').read_text()
def function(name):
 return re.search(r'^'+name+r'\(\) \{\n.*?^\}',source,re.M|re.S).group()
stages=source[source.index('run_stage "前提检查"'):source.index('run_stage "验证构建产物"')]
rows=[]
with tempfile.TemporaryDirectory() as tmp:
 for failing in ('0','1'):
  log=Path(tmp)/('events'+failing)
  env=os.environ.copy();env.update(EVENTS=str(log),FAIL_BUILD=failing,SRC_BRANCH='fixture')
  harness='set -Eeuo pipefail\n'+function('cleanup')+'\n'+function('run_stage')+r'''
trap cleanup EXIT
sudo() { echo "sudo $*" >> "$EVENTS"; }
check_prereqs() { echo prereqs >> "$EVENTS"; }
sync_sources() { echo sync >> "$EVENTS"; }
materialize_tinygrad() { echo tinygrad >> "$EVENTS"; }
materialize_gitlinks() { echo gitlinks >> "$EVENTS"; }
rebuild_native() { echo build >> "$EVENTS"; [ "$FAIL_BUILD" != 1 ]; }
compile_driving_pkl() { echo model >> "$EVENTS"; }
'''+stages
  p=subprocess.run(['bash','-c',harness],env=env,capture_output=True,text=True)
  events=log.read_text().splitlines()
  assert p.returncode==int(failing),(failing,p.returncode,p.stderr)
  assert events[0]=='prereqs' and events[-1]=='sudo systemctl start comma',events
  assert events.count('sudo systemctl stop comma')==1,events
  assert events.index('prereqs') < events.index('sudo systemctl stop comma') < events.index('sync') < events.index('build'),events
  assert ('gitlinks' in events) and ('build' in events)
  rows.append({'build_failure':bool(int(failing)),'exit':p.returncode,'events':events})
print(json.dumps({'pass':True,'scope':'actual stage and cleanup shell with no-op systemctl; no hardware','cases':rows},indent=2))
