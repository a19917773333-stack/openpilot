import sys, bisect
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:-2]; lo,hi=float(sys.argv[-2]),float(sys.argv[-1])
cars=[]; ctrls=[]; ccs=[]; sds=[]
seen=set()
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime
    if t in seen: continue
    seen.add(t); w=m.which()
    if w=='carState':
      s=m.carState; cars.append((t,s))
    elif w=='controlsState': ctrls.append((t,m.controlsState))
    elif w=='carControl': ccs.append((t,m.carControl))
    elif w=='selfdriveState': sds.append((t,m.selfdriveState))
cars.sort(key=lambda x:x[0]); ctrls.sort(key=lambda x:x[0]); ccs.sort(key=lambda x:x[0]); sds.sort(key=lambda x:x[0])
t0=cars[0][0]
def near(arr,t):
  if not arr:return None
  ts=[x[0] for x in arr]; i=bisect.bisect_left(ts,t)
  if i<=0:return arr[0][1]
  if i>=len(arr):return arr[-1][1]
  return arr[i-1][1] if t-ts[i-1]<=ts[i]-t else arr[i][1]
sub=[x for x in cars if lo*1e9<=x[0]-t0<=hi*1e9]
last=-1
for t,s in sub:
  tt=(t-t0)/1e9
  if tt-last<0.10: continue
  last=tt; c= near(ctrls,t); cc=near(ccs,t); sd=near(sds,t)
  latstate=c.lateralControlState.which() if c else '?'
  target=0.0
  if latstate=='angleState': target=c.lateralControlState.angleState.steeringAngleDesiredDeg
  print(f"t={tt:7.2f} v={s.vEgo:5.2f} ang={s.steeringAngleDeg:7.1f} rate={getattr(s,'steeringRateDeg',0):7.1f} trq={s.steeringTorque:5.2f} eps={s.steeringTorqueEps:5.2f} press={int(s.steeringPressed)} L={int(s.leftBlinker)} R={int(s.rightBlinker)} yaw={s.yawRate:7.4f} lat={int(cc.latActive) if cc else -1} act={int(sd.active) if sd else -1} DC={c.desiredCurvature if c else 0:8.4f} latState={latstate} target={target:6.1f} cmdAng={cc.actuators.steeringAngleDeg if cc else 0:6.1f} cmdTrq={cc.actuators.torque if cc else 0:6.2f}")
