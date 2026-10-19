import sys
from openpilot.tools.lib.logreader import LogReader
for p in sys.argv[1:]:
  for m in LogReader(p):
    if 'Odometry' in m.which() or 'odometry' in m.which():
      print(m.which(), list(m.to_dict().keys()))
      print(m.to_dict())
      raise SystemExit
