import sys
from openpilot.tools.lib.logreader import LogReader
for p in sys.argv[1:]:
  for m in LogReader(p):
    if m.which()=='carParams':
      s=m.carParams
      print('carFingerprint', s.carFingerprint)
      print('steerRatio', s.steerRatio)
      print('mass', s.mass, 'wheelbase', s.wheelbase)
      raise SystemExit
