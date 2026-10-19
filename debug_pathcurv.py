import sys, numpy as np
from openpilot.tools.lib.logreader import LogReader
from openpilot.sunnypilot.modeld_v2.constants import ModelConstants, Plan
from openpilot.sunnypilot.modeld_v2.turn_curvature import _path_curvature, _lane_state
from openpilot.cereal import custom
TurnDirection=custom.ModelDataV2SP.TurnDirection
p=sys.argv[1:-1]; target=float(sys.argv[-1]); ti=np.asarray(ModelConstants.T_IDXS,dtype=float)
cars=[]; dmds=[]
seen=set()
for path in p:
  for m in LogReader(path):
    if m.logMonoTime in seen: continue
    seen.add(m.logMonoTime)
    if m.which()=='carState': cars.append((m.logMonoTime,m.carState))
    elif m.which()=='drivingModelData': dmds.append((m.logMonoTime,m.drivingModelData))
cars.sort(); dmds.sort(); t0=cars[0][0]; t=t0+int(target*1e9)
d=min(dmds,key=lambda x:abs(x[0]-t))[1]; c=min(cars,key=lambda x:abs(x[0]-t))[1]
tr=np.zeros((len(ti),15),dtype=np.float32)
xc=np.array(d.path.xCoefficients); yc=np.array(d.path.yCoefficients)
x=np.polynomial.polynomial.polyval(ti,xc); y=np.polynomial.polynomial.polyval(ti,yc)
dx=np.polynomial.polynomial.polyval(ti,np.polynomial.polynomial.polyder(xc)); dy=np.polynomial.polynomial.polyval(ti,np.polynomial.polynomial.polyder(yc))
tr[:,Plan.POSITION][:,0]=x; tr[:,Plan.POSITION][:,1]=y; tr[:,Plan.T_FROM_CURRENT_EULER][:,2]=np.arctan2(dy,dx)
print('v',c.vEgo,'ang',c.steeringAngleDeg,'action',d.action.desiredCurvature,'lanemeta',d.laneLineMeta)
print('pathcurv',_path_curvature([tr],ti,1.0))
