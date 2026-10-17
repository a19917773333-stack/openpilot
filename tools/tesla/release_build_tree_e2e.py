#!/usr/bin/env python3
"""Execute release Git checkout/materialization against local fixture repositories."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from tools.release import release_lib


def git(where, *args):
  return subprocess.check_output(['git', '-C', str(where), *args], text=True).strip()


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--output', type=Path, required=True)
  args = parser.parse_args()
  source = (ROOT/'tools/release/device_release.sh').read_text()
  assert 'SRC=/data/build' in source, 'release still builds in the live runtime tree'
  definitions = source[:source.index('\nrun_stage "前提检查"')]
  rows = []
  with tempfile.TemporaryDirectory() as directory:
    tmp = Path(directory)
    module, repo, build, runtime = [tmp/name for name in ('panda-source', 'source', 'build', 'runtime')]
    for path in (module, repo):
      path.mkdir()
      git(path, 'init', '-q', '-b', 'dev-sp-chipmunk')
      git(path, 'config', 'user.name', 'fixture')
      git(path, 'config', 'user.email', 'fixture@invalid')
    (module/'firmware-input').write_text('pinned source, no hardware')
    git(module, 'add', '.')
    git(module, 'commit', '-qm', 'module')
    pin = git(module, 'rev-parse', 'HEAD')
    (repo/'.gitmodules').write_text(f'[submodule "panda"]\n path = panda\n url = {module}\n')
    (repo/'launch_env.sh').write_text((ROOT/'launch_env.sh').read_text())
    (repo/'source-input').write_text('expected-source')
    git(repo, 'add', '.')
    git(repo, 'update-index', '--add', '--cacheinfo', f'160000,{pin},panda')
    git(repo, 'commit', '-qm', 'source')
    runtime.mkdir()
    (tmp/'device-model').write_text('comma tici')
    sentinel = runtime/'dirty-user-file'
    sentinel.write_text('must survive')
    env = {**os.environ, 'TEST_BUILD': str(build), 'TEST_URL': str(repo), 'TEST_EVENTS': str(tmp/'events'),
           'SUNNYPILOT_HARDWARE_PROFILE': 'c3xl', 'RELEASE_BRANCH': 'lean-tesla-release-c3',
           'SKIP_CAPNP_REGEN': '1', 'SKIP_TINYGRAD_COMPILE': '1',
           'GIT_ALLOW_PROTOCOL': 'file', 'SUNNYPILOT_HARDWARE_MODEL_FILE': str(tmp/'device-model')}
    script = definitions + r'''
SRC="$TEST_BUILD"
REPO_URL="$TEST_URL"
trap - EXIT
check_hardware_profile
sync_sources
materialize_gitlinks
echo dirty > "$SRC/stale-output"
echo dirty > "$SRC/panda/stale-output"
sync_sources
materialize_gitlinks
scons() {
  [ -z "${SKIP_CAPNP_REGEN+x}" ] && [ -z "${SKIP_TINYGRAD_COMPILE+x}" ] || return 1
  echo "$*|$C3XL_IFE_ROAD_SIZE" >> "$TEST_EVENTS"
  [ "${FAIL_BUILD:-0}" != 1 ]
}
rebuild_native
'''
    result = subprocess.run(['bash', '-c', script], env=env, text=True, capture_output=True)
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert (build/'source-input').read_text() == 'expected-source'
    assert not (build/'stale-output').exists() and not (build/'panda/stale-output').exists()
    assert git(build/'panda', 'rev-parse', 'HEAD') == pin
    assert sentinel.read_text() == 'must survive'
    events = (tmp/'events').read_text().splitlines()
    assert len(events) == 2 and '-j1' in events[0] and '-j8' in events[1], events
    assert all('--minimal' in e and '1344x760' in e for e in events), events
    rows.append({'case': 'clean source and pinned submodule, runtime preserved, correct geometry', 'pass': True,
                 'source': git(repo, 'rev-parse', 'HEAD'), 'panda_pin': pin, 'scons_requests': events})
    failure = subprocess.run(['bash', '-c', definitions + '\nSRC="$TEST_BUILD"\ntrap - EXIT\nscons() { return 1; }\nrebuild_native\n'],
                             env=env, text=True, capture_output=True)
    assert failure.returncode != 0
    rows.append({'case': 'build failure rejects release', 'pass': True, 'exit': failure.returncode})
    for path in release_lib.ARTIFACT_PATHS:
      target = build/path
      target.parent.mkdir(parents=True, exist_ok=True)
      header = bytearray(64)
      header[:6] = b'\x7fELF\x02\x01'
      header[18:20] = (183).to_bytes(2, 'little')
      target.write_bytes(b'firmware-fixture' if '.bin' in target.name else header)
    helper = build/'tools/release/release_lib.py'
    helper.parent.mkdir(parents=True, exist_ok=True)
    helper.write_bytes((ROOT/'tools/release/release_lib.py').read_bytes())
    for path in ['openpilot/cereal/schemas.py', *release_lib.load_schema_registry(ROOT)]:
      target = build/path
      target.parent.mkdir(parents=True, exist_ok=True)
      target.write_bytes((ROOT/path).read_bytes())
    (build/'openpilot/cereal/gen/cpp').mkdir(parents=True, exist_ok=True)
    model = build/'openpilot/selfdrive/modeld/models/driving_tinygrad.pkl'
    model.parent.mkdir(parents=True, exist_ok=True)
    Path(str(model)+'.chunkmanifest').write_text('2')
    chunks = [Path(str(model)+f'.chunk{i:02}of02') for i in (1, 2)]
    for chunk in chunks:
      chunk.write_bytes(b'chunk-fixture')
    code = definitions + '\nSRC="$TEST_BUILD"\ntrap - EXIT\ncheck_artifacts\n'
    env['PYTHONPATH'] = str(ROOT)
    env['PATH'] = str(Path(sys.executable).parent)+os.pathsep+env['PATH']
    for failure, path in [('none', None), ('missing-DOS', build/'panda/board/obj/panda.bin.signed'),
                          ('missing-legacy-MPC', build/next(p for p in release_lib.ARTIFACT_PATHS if 'c_generated_code_fallback/acados_ocp_solver_pyx' in p)),
                          ('missing-model-tail', chunks[-1]), ('PC-built-ELF', build/'openpilot/common/libparams_c.so')]:
      saved = path.read_bytes() if path else None
      if path:
        if failure == 'PC-built-ELF':
          path.write_bytes(saved+b'/.comma/params')
        else:
          path.unlink()
      check = subprocess.run(['bash', '-c', code], env=env, text=True, capture_output=True)
      if path:
        path.write_bytes(saved)
      assert (check.returncode == 0) == (failure == 'none'), (failure, check.stdout, check.stderr)
      rows.append({'case': 'artifact-gate-'+failure, 'pass': True, 'exit': check.returncode})
  report = {'pass': True, 'scope': 'actual Git and shell workflow, local source/submodule fixtures and synthetic scons boundary; no native/device build', 'cases': rows}
  args.output.parent.mkdir(parents=True, exist_ok=True)
  args.output.write_text(json.dumps(report, indent=2)+'\n')
  print(json.dumps(report, indent=2))


if __name__ == '__main__':
  main()
