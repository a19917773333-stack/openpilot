import sys, bisect, math
import numpy as np
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:-1]; target=float(sys.argv[-1])
cars=[]; dmds=[]; seen=set()
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime
    if t in seen: continue
    seen.add(t)
    if m.which()=='carState': cars.append((t,m.carState))
    elif m.which()=='drivingModelData': dmds.append((t,m.drivingModelData))
cars.sort(key=lambda x:x[0]); dmds.sort(key=lambda x:x[0]); t0=cars[0][0]
t=t0+int(target*1e9)
d=min(dmds,key=lambda x:abs(x[0]-t))[1]
f=d.path; cx=np.array(f.xCoefficients); cy=np.array(f.yCoefficients)
print('actionDC',d.action.desiredCurvature,'lane',d.laneLineMeta)
for tq in (0.5,1,2,3,4,5,7,10):
  x=np.polynomial.polynomial.polyval(tq,cx); y=np.polynomial.polynomial.polyval(tq,cy)
  dx=np.polynomial.polynomial.polyval(tq,np.polynomial.polynomial.polyder(cx)); dy=np.polynomial.polynomial.polyval(tq,np.polynomial.polynomial.polyder(cy))
  ddx=np.polynomial.polynomial.polyval(tq,np.polynomial.polynomial.polyder(cx,2)); ddy=np.polynomial.polynomial.polyval(tq,np.polynomial.polynomial.polyder(cy,2))
  den=(dx*dx+dy*dy)**1.5; k=(dx*ddy-dy*ddx)/den if den>1e-9 else 0
  print(f't={tq:4.1f} x={x:7.2f} y={y:7.2f} yaw={math.degrees(math.atan2(dy,dx)):7.2f} k={k:8.4f}')
