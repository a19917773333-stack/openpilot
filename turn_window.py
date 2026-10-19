import sys, bisect
from openpilot.tools.lib.logreader import LogReader
paths = sys.argv[1:-2]
lo, hi = float(sys.argv[-2]), float(sys.argv[-1])
rows = []
for path in paths:
    for msg in LogReader(path):
        t = msg.logMonoTime; w = msg.which()
        if w == 'carState':
            s=msg.carState
            rows.append({'t':t,'type':'car','v':s.vEgo,'ang':s.steeringAngleDeg,'trq':s.steeringTorque,
                         'press':s.steeringPressed,'lb':s.leftBlinker,'rb':s.rightBlinker})
        elif w == 'controlsState':
            s=msg.controlsState
            rows.append({'t':t,'type':'ctrl','dc':s.desiredCurvature,'curv':s.curvature})
        elif w == 'carControl':
            s=msg.carControl
            rows.append({'t':t,'type':'cc','lat':s.latActive,'torque':s.actuators.torque,'sang':s.actuators.steeringAngleDeg})
        elif w == 'drivingModelData':
            rows.append({'t':t,'type':'md','mdc':msg.drivingModelData.action.desiredCurvature,'big':msg.drivingModelData.big})
        elif w == 'selfdriveState':
            s=msg.selfdriveState
            rows.append({'t':t,'type':'sd','active':s.active,'enabled':s.enabled,'alert':s.alertText1,'alert2':s.alertText2})
rows.sort(key=lambda x:x['t'])
t0=rows[0]['t']
sub=[r for r in rows if lo*1e9 <= r['t']-t0 <= hi*1e9]
print('window', lo, hi, 'rows', len(sub))
car=[r for r in sub if r['type']=='car']
def near(typ,t):
    arr=[r for r in sub if r['type']==typ]
    if not arr: return {}
    ts=[r['t'] for r in arr]; i=bisect.bisect_left(ts,t)
    j=min(max(i,1),len(arr)-1)
    return arr[j-1] if t-ts[j-1] <= ts[j]-t else arr[j]
last=-1
for r in car:
    tt=(r['t']-t0)/1e9
    if tt-last < 0.20: continue
    last=tt
    c=near('ctrl',r['t']); cc=near('cc',r['t']); m=near('md',r['t']); sd=near('sd',r['t'])
    print(f"t={tt:7.2f} v={r['v']:5.2f} ang={r['ang']:7.1f} trq={r['trq']:6.2f} press={int(r['press'])} L={int(r['lb'])} R={int(r['rb'])} lat={int(cc.get('lat',0))} DC={c.get('dc',0):8.4f} mDC={m.get('mdc',0):8.4f} ccTrq={cc.get('torque',0):6.2f} active={int(sd.get('active',0))} alert={sd.get('alert','')}")
ev=[r for r in sub if r['type']=='sd' and (r.get('alert') or r.get('alert2'))]
seen=set()
for r in ev:
    k=(r.get('alert'),r.get('alert2'))
    if k not in seen:
        seen.add(k); print('ALERT', round((r['t']-t0)/1e9,2), k)
