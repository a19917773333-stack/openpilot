import sys, bisect, math
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.turn_curvature import path_curvature_for_turn
from openpilot.cereal import custom
TurnDirection=custom.ModelDataV2SP.TurnDirection
paths=sys.argv[1:-2]; lo,hi=float(sys.argv[-2]),float(sys.argv[-1])
cars=[]; dmds=[]; seen=set()
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime
    if t in seen: continue
    seen.add(t)
    if m.which()=='carState': cars.append((t,m.carState))
    elif m.which()=='drivingModelData': dmds.append((t,m.drivingModelData))
cars.sort(key=lambda x:x[0]); dmds.sort(key=lambda x:x[0]); t0=cars[0][0]
t_idxs=np.asarray(ModelConstants.T_IDXS,dtype=float)
def make_plan(d):
  c=np.array(d.path.yCoefficients,dtype=float); xcp=np.array(d.path.xCoefficients,dtype=float)
  tr=np.zeros((len(t_idxs),15),dtype=np.float32)
  xs=np.polynomial.polynomial.polyval(t_idxs,xcp); ys=np.polynomial.polynomial.polyval(t_idxs,c)
  dx=np.polynomial.polynomial.polyval(t_idxs,np.polynomial.polynomial.polyder(xcp)); dy=np.polynomial.polynomial.polyval(t_idxs,np.polynomial.polynomial.polyder(c))
  tr[:,Plan.POSITION][:,0]=xs; tr[:,Plan.POSITION][:,1]=ys
  tr[:,Plan.T_FROM_CURRENT_EULER][:,2]=np.arctan2(dy,dx)
  return [tr]
def angle(k):
  return math.degrees(math.atan(k*2.89))*12.0 if abs(k)<0.4 else math.copysign(200,k)
prev=0.0; last=0.0; dt=0.0
for t,s in cars:
  tr=(t-t0)/1e9
  if tr<lo: continue
  if tr>hi: break
  d=min(dmds,key=lambda x:abs(x[0]-t))[1]
  direction=TurnDirection.turnLeft if s.steeringAngleDeg>0 else TurnDirection.turnRight
  plan=make_plan(d)
  out=path_curvature_for_turn(plan,t_idxs,float(s.vEgo),0.5,direction,
        action_curvature=float(d.action.desiredCurvature),prev_curvature=prev,dt=max(0.01,tr-last),
        lane_lines=None,lane_probs=None,steer_ratio=12.0,wheelbase=2.89)
  prev=out if out is not None else 0.0; last=tr
  if tr-lo < 0.05 or (tr*2)%2<0.06:
    print(f't={tr:7.2f} v={s.vEgo:5.2f} actual={s.steeringAngleDeg:7.1f} newTarget={angle(prev):7.1f} DC={prev:8.4f} oldDC={d.action.desiredCurvature:8.4f}')
