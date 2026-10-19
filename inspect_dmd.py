import sys
from openpilot.tools.lib.logreader import LogReader
path=sys.argv[1]; first={}
for msg in LogReader(path):
  first.setdefault(msg.which(),msg)
  if all(k in first for k in ('carState','controlsState','carControl','drivingModelData')): break
for name in ('drivingModelData',):
  d=first[name].to_dict()[name]
  print(name, list(d.keys()))
  for k,v in d.items(): print(' ',k, type(v).__name__, str(v)[:300])
