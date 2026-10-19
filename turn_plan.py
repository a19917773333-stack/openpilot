import sys, bisect, math
import numpy as np
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:-2]; lo,hi=float(sys.argv[-2]),float(sys.argv[-1])
cars=[]; ctrls=[]; dmds=[]; ccs=[]; sds=[]
seen=set()
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime
    if t in seen: continue
    seen.add(t); w=m.which()
    if w=='carState': cars.append((t,m.carState))
    elif w=='controlsState': ctrls.append((t,m.controlsState))
    elif w=='drivingModelData': dmds.append((t,m.drivingModelData))
    elif w=='carControl': ccs.append((t,m.carControl))
    elif w=='selfdriveState': sds.append((t,m.selfdriveState))
for a in (cars,ctrls,dmds,ccs,sds): a.sort(key=lambda x:x[0])
t0=cars[0][0]
def near(arr,t):
  if not arr:return None
  ts=[x[0] for x in arr]; i=bisect.bisect_left(ts,t)
  if i<=0:return arr[0][1]
  if i>=len(arr):return arr[-1][1]
  return arr[i-1][1] if t-ts[i-1]<=ts[i]-t else arr[i][1]
def plan_curv(dmd, tq):
  f=dmd.path
  cx=np.array(f.xCoefficients,dtype=float); cy=np.array(f.yCoefficients,dtype=float)
  # coeffs are ascending powers of time
  x=np.polyval(cx[::-1],tq); y=np.polyval(cy[::-1],tq)
  dx=np.polyval(np.polyder(cx)[::-1],tq); dy=np.polyval(np.polyder(cy)[::-1],tq)
  ddx=np.polyval(np.polyder(cx,2)[::-1],tq); ddy=np.polyval(np.polyder(cy,2)[::-1],tq)
  den=(dx*dx+dy*dy)**1.5
  k=(dx*ddy-dy*ddx)/den if den>1e-9 else 0.0
  return x,y,k
print('t      v    ang   press act lat  actionDC  planK@1  @2  @3  @4  @5   leftY rightY lP rP')
last=-1
for t,s in cars:
  tt=(t-t0)/1e9
  if not(lo<=tt<=hi): continue
  if tt-last<0.20: continue
  last=tt
  c=near(ctrls,t); cc=near(ccs,t); sd=near(sds,t); d=near(dmds,t)
  ks=[]
  for tq in (1,2,3,4,5): ks.append(plan_curv(d,tq)[2] if d else 0.0)
  lm=d.laneLineMeta if d else None
  print(f"{tt:7.2f} {s.vEgo:5.2f} {s.steeringAngleDeg:6.1f} {int(s.steeringPressed)} {int(sd.active) if sd else -1} {int(cc.latActive) if cc else -1} {c.desiredCurvature if c else 0:8.4f}  "+' '.join(f'{k:7.4f}' for k in ks)+f"  {lm.leftY if lm else 0:6.2f} {lm.rightY if lm else 0:6.2f} {lm.leftProb if lm else 0:4.2f} {lm.rightProb if lm else 0:4.2f}")
