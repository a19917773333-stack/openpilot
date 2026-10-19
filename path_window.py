import sys
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:]
rows=[]
for p in paths:
  for m in LogReader(p):
    if m.which()=='drivingModelData':
      s=m.drivingModelData; rows.append((m.logMonoTime,s))
rows.sort()
t0=rows[0][0]
def ev(c,x):
  return sum(v*(x**i) for i,v in enumerate(c))
for t,s in rows:
  tt=(t-t0)/1e9
  if not 172 <= tt <= 186: continue
  y=s.path.yCoefficients; x=s.path.xCoefficients
  print(f"t={tt:7.2f} big={int(s.big)} DC={s.action.desiredCurvature:8.4f} a={s.action.desiredAcceleration:6.2f} leftY={s.laneLineMeta.leftY:6.2f} rightY={s.laneLineMeta.rightY:6.2f} pL={s.laneLineMeta.leftProb:4.2f} pR={s.laneLineMeta.rightProb:4.2f} y5={ev(y,5):6.2f} y10={ev(y,10):6.2f} y20={ev(y,20):7.2f} y30={ev(y,30):7.2f} x30={ev(x,30):6.2f}")
