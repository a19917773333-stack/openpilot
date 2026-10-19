import sys, bisect, csv, json
from openpilot.tools.lib.logreader import LogReader
paths=sys.argv[1:-1]; out=sys.argv[-1]
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
  if not arr: return None
  ts=[x[0] for x in arr]; i=bisect.bisect_left(ts,t)
  if i<=0: return arr[0][1]
  if i>=len(arr): return arr[-1][1]
  return arr[i-1][1] if t-ts[i-1]<=ts[i]-t else arr[i][1]
def target_angle(c):
  if c is None: return 0.0
  ls=c.lateralControlState
  return float(ls.angleState.steeringAngleDesiredDeg) if ls.which()=='angleState' else 0.0
windows=[('left',205.0,215.0),('right',216.0,225.5)]
rows=[]; summary={}
for name,lo,hi in windows:
  wr=[]
  for t,s in cars:
    tr=(t-t0)/1e9
    if not (lo<=tr<=hi): continue
    c=near(ctrls,t); cc=near(ccs,t); d=near(dmds,t); sd=near(sds,t)
    lm=d.laneLineMeta if d is not None else None
    row={
      'direction':name,'t_route':round(tr,3),'t_turn':round(tr-lo,3),'v_ego':round(float(s.vEgo),3),
      'steering_angle_deg':round(float(s.steeringAngleDeg),2),
      'steering_target_deg':round(target_angle(c),2),
      'desired_curvature':round(float(c.desiredCurvature),5) if c is not None else 0.0,
      'model_curvature':round(float(d.action.desiredCurvature),5) if d is not None else 0.0,
      'driver_pressed':int(bool(s.steeringPressed)),
      'selfdrive_active':int(bool(sd.active)) if sd is not None else -1,
      'lat_active':int(bool(cc.latActive)) if cc is not None else -1,
      'left_lane_prob':round(float(lm.leftProb),3) if lm is not None else 0.0,
      'right_lane_prob':round(float(lm.rightProb),3) if lm is not None else 0.0,
      'left_lane_y':round(float(lm.leftY),3) if lm is not None else 0.0,
      'right_lane_y':round(float(lm.rightY),3) if lm is not None else 0.0,
    }
    rows.append(row); wr.append(row)
  peak=max(wr,key=lambda r:abs(r['steering_angle_deg']))
  pressed=[r for r in wr if r['driver_pressed']]
  summary[name]={
    'window_s':[lo,hi],
    'peak_angle_deg':peak['steering_angle_deg'],
    'peak_angle_t_route_s':peak['t_route'],
    'peak_target_deg':peak['steering_target_deg'],
    'peak_desired_curvature':peak['desired_curvature'],
    'peak_model_curvature':peak['model_curvature'],
    'first_driver_press_t_route_s':pressed[0]['t_route'] if pressed else None,
    'first_driver_press_angle_deg':pressed[0]['steering_angle_deg'] if pressed else None,
    'last_driver_press_t_route_s':pressed[-1]['t_route'] if pressed else None,
    'max_lane_conf_before_peak':{
      'left':max(r['left_lane_prob'] for r in wr if r['t_route']<=peak['t_route']),
      'right':max(r['right_lane_prob'] for r in wr if r['t_route']<=peak['t_route']),
    },
    'peak_v_ego':peak['v_ego'],
  }
with open(out,'w',newline='',encoding='utf-8') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print(json.dumps(summary,indent=2,ensure_ascii=False))
print('rows',len(rows),'saved',out)
