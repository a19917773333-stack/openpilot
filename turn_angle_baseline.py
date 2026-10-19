import sys, os, bisect
from openpilot.tools.lib.logreader import LogReader

paths = sys.argv[1:]
cars=[]; ctrls=[]; ccs=[]; sds=[]
seen=set()
for p in paths:
  for m in LogReader(p):
    t=m.logMonoTime
    if t in seen: continue
    seen.add(t); w=m.which()
    if w=='carState':
      s=m.carState; cars.append((t,s.vEgo,s.steeringAngleDeg,s.steeringTorque,s.steeringPressed,s.leftBlinker,s.rightBlinker,s.yawRate))
    elif w=='controlsState':
      s=m.controlsState; ctrls.append((t,s.desiredCurvature,s.curvature))
    elif w=='carControl':
      s=m.carControl; ccs.append((t,s.latActive,s.actuators.torque,s.actuators.steeringAngleDeg))
    elif w=='selfdriveState':
      s=m.selfdriveState; sds.append((t,s.active,s.enabled,s.alertText1))
cars.sort(); ctrls.sort(); ccs.sort(); sds.sort()
if not cars: raise SystemExit('no carState')
t0=cars[0][0]; ct=[x[0] for x in ctrls]; mt=[x[0] for x in ccs]; st=[x[0] for x in sds]
def near(ts,arr,t):
  i=bisect.bisect_left(ts,t)
  if i<=0: return arr[0]
  if i>=len(arr): return arr[-1]
  return arr[i-1] if t-ts[i-1] <= ts[i]-t else arr[i]
# contiguous turn episodes based on |angle| >= 18deg, vehicle moving, gaps <= 0.8s
eps=[]; cur=None
for r in cars:
  t,v,ang,trq,press,lb,rb,yr=r
  active = abs(ang)>=18 and v>=0.5
  if active:
    if cur is None or t-cur[-1][0] > 1_200_000_000:
      if cur: eps.append(cur)
      cur=[r]
    else: cur.append(r)
  elif cur is not None and t-cur[-1][0] > 1_200_000_000:
    eps.append(cur); cur=None
if cur: eps.append(cur)
# keep episodes >=1.0 s and a real angle change
eps=[e for e in eps if (e[-1][0]-e[0][0])>=1_000_000_000 and max(map(abs,(x[2] for x in e)))-min(map(abs,(x[2] for x in e)))>=8]
print('route span', (cars[0][0]-t0)/1e9, (cars[-1][0]-t0)/1e9, 'car msgs', len(cars), 'episodes', len(eps))
for k,e in enumerate(eps):
  mn=min(e,key=lambda x:x[2]); mx=max(e,key=lambda x:x[2]); peak=max(e,key=lambda x:abs(x[2]))
  sign='LEFT' if peak[2]>0 else 'RIGHT'
  c=near(ct,ctrls,peak[0]); cc=near(mt,ccs,peak[0]); sd=near(st,sds,peak[0])
  print(f"\nEP{k} {sign} t={(e[0][0]-t0)/1e9:7.2f}-{(e[-1][0]-t0)/1e9:7.2f} dur={(e[-1][0]-e[0][0])/1e9:4.1f}s v={min(x[1] for x in e):4.1f}-{max(x[1] for x in e):4.1f} ang={min(x[2] for x in e):6.1f}..{max(x[2] for x in e):6.1f} peak={peak[2]:6.1f}@{(peak[0]-t0)/1e9:7.2f}s lat={int(cc[1])} active={int(sd[1])} DC={c[1]:7.4f} trq={peak[3]:5.1f} press={int(peak[4])} L={int(peak[5])} R={int(peak[6])}")
  # print every ~0.5s plus start/end/peak neighbors
  rows=[e[0]]+[x for i,x in enumerate(e[1:-1],1) if (x[0]-e[i-1][0])>450_000_000]+[e[-1]]
  if peak not in rows: rows.append(peak); rows.sort()
  for r in rows:
    c=near(ct,ctrls,r[0]); cc=near(mt,ccs,r[0]); sd=near(st,sds,r[0])
    print(f"  t={(r[0]-t0)/9e9:7.2f} v={r[1]:4.1f} ang={r[2]:6.1f} trq={r[3]:5.1f} press={int(r[4])} L={int(r[5])} R={int(r[6])} lat={int(cc[1])} act={int(sd[1])} DC={c[1]:7.4f} cmdTrq={cc[2]:5.1f}")
