import sys, bisect
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:-2]; lo,hi=float(sys.argv[-2]),float(sys.argv[-1])
rows=[]
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime; w=m.which()
    if w=='carState':
      s=m.carState; rows.append({'t':t,'k':'car','v':s.vEgo,'ang':s.steeringAngleDeg,'trq':s.steeringTorque,'press':s.steeringPressed,'lb':s.leftBlinker,'rb':s.rightBlinker})
    elif w=='controlsState':
      s=m.controlsState; ls=s.lateralControlState
      target = ls.angleState.steeringAngleDesiredDeg if ls.which()=='angleState' else 0.0
      rows.append({'t':t,'k':'ctrl','dc':s.desiredCurvature,'curv':s.curvature,'target':target})
    elif w=='carControl':
      s=m.carControl; rows.append({'t':t,'k':'cc','lat':s.latActive,'cmd':s.actuators.steeringAngleDeg,'torque':s.actuators.torque})
    elif w=='selfdriveState':
      s=m.selfdriveState; rows.append({'t':t,'k':'sd','active':s.active,'alert':s.alertText1})
rows.sort(key=lambda x:x['t']); t0=rows[0]['t']
sub=[r for r in rows if lo*1e9<=r['t']-t0<=hi*1e9]
def near(k,t):
  arr=[r for r in sub if r['k']==k]; ts=[r['t'] for r in arr]
  i=bisect.bisect_left(ts,t); j=min(max(i,1),len(arr)-1)
  return arr[j-1] if t-ts[j-1]<=ts[j]-t else arr[j]
last=-1
for r in [x for x in sub if x['k']=='car']:
  tt=(r['t']-t0)/1e9
  if tt-last<0.2: continue
  last=tt; c=near('ctrl',r['t']); cc=near('cc',r['t']); sd=near('sd',r['t'])
  print(f"t={tt:7.2f} v={r['v']:5.2f} actual={r['ang']:7.1f} cmd={cc.get('cmd',0):7.1f} target={c.get('target',0):7.1f} trq={r['trq']:5.2f} press={int(r['press'])} L={int(r['lb'])} lat={int(cc.get('lat',0))} DC={c.get('dc',0):8.4f} active={int(sd.get('active',0))} {sd.get('alert','')}")
