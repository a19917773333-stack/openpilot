import sys, math
from openpilot.tools.lib.logreader import LogReader
rows=[]
for p in sys.argv[1:]:
  for m in LogReader(p):
    if m.which()=='cameraOdometry':
      s=m.cameraOdometry
      rows.append((m.logMonoTime, list(s.rot), s.timestampEof))
rows.sort()
t0=rows[0][0]
cum=0.0; last_print=-1
for t,rot,eof in rows:
  tt=(t-t0)/1e9
  # rotation vector magnitude around z, small-angle approximation
  cum += rot[2]
  if 170 <= tt <= 190 and tt-last_print >= 0.25:
    last_print=tt
    print(f"t={tt:7.2f} yaw_cum={math.degrees(cum):8.2f} deg rz={rot[2]:+.5f}")
print('total', math.degrees(cum))
