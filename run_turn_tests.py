import importlib, inspect, traceback
mod = importlib.import_module('openpilot.sunnypilot.modeld_v2.tests.test_turn_curvature')
names = [n for n in dir(mod) if n.startswith('test_') and callable(getattr(mod, n))]
failed = []
for name in names:
  try:
    getattr(mod, name)()
    print('PASS', name)
  except Exception:
    failed.append(name)
    print('FAIL', name)
    traceback.print_exc()
print(f'{len(names)-len(failed)}/{len(names)} passed')
raise SystemExit(bool(failed))
