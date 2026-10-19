import sys, math, bisect
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:-2]; lo,hi=float(sys.argv[-2]),float(sys.argv[-1])
od=[]; cars=[]; seen=set()
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime
    if t in seen: continue
    seen.add(t); w=m.which()
    if w=='cameraOdometry': od.append((t,m.cameraOdometry))
    elif w=='carState': cars.append((t,m.carState))
od.sort(key=lambda x:x[0]); cars.sort(key=lambda x:x[0]); t0=cars[0][0]; cts=[x[0] for x in cars]
yaw=0.0
for t,o in od:
  tr=(t-t0)/1e9
  if tr<lo: continue
  if tr>hi: break
  yaw += float(o.rot[2])
  i=bisect.bisect_left(cts,t); s=cars[max(0,min(i,len(cars)-1))][1]
  print(f"t={tr:7.2f} v={s.vEgo:5.2f} ang={s.steeringAngleDeg:7.1f} yawInt={math.degrees(yaw):7.1f} rotz={o.rot[2]:8.5f}")
