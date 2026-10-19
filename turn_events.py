import sys, bisect, glob
from openpilot.tools.lib.logreader import LogReader

paths = sys.argv[1:]
car, ctrl, mdl = [], [], []
for path in paths:
    for msg in LogReader(path):
        t = msg.logMonoTime
        w = msg.which()
        if w == 'carState':
            s = msg.carState
            car.append((t, s.vEgo, s.steeringAngleDeg, s.steeringTorque, s.yawRate, s.leftBlinker, s.rightBlinker, s.leftBlindspot, s.rightBlindspot))
        elif w == 'controlsState':
            s = msg.controlsState
            ctrl.append((t, s.desiredCurvature, s.curvature, s.deprecated.steeringAngleDesiredDeg))
        elif w == 'drivingModelData':
            s = msg.drivingModelData
            mdl.append((t, s.action.desiredCurvature, s.action.desiredAcceleration, s.big))

car.sort(); ctrl.sort(); mdl.sort()
ct = [x[0] for x in ctrl]; mt = [x[0] for x in mdl]
def nearest(ts, arr, t):
    i = bisect.bisect_left(ts, t)
    if i <= 0: return arr[0]
    if i >= len(arr): return arr[-1]
    return arr[i-1] if t-ts[i-1] <= ts[i]-t else arr[i]

def describe(t0, t1, side):
    rows = [x for x in car if t0 <= x[0] <= t1]
    if not rows: return
    v = [x[1] for x in rows]; ang = [x[2] for x in rows]; trq = [x[3] for x in rows]; yr = [x[4] for x in rows]
    c = [nearest(ct, ctrl, x[0]) for x in rows]
    dc = [x[1] for x in c]; dsa = [x[3] for x in c]
    m = [nearest(mt, mdl, x[0]) for x in rows]
    mdc = [x[1] for x in m]; big = [x[3] for x in m]
    print(f'{side:5s} {t0/1e9:.2f}-{t1/1e9:.2f} dur={((t1-t0)/1e9):4.1f}s v={min(v):4.1f}-{max(v):4.1f} steer={min(ang):6.1f}..{max(ang):6.1f} max|ang|={max(map(abs,ang)):5.1f} torque={min(trq):5.1f}..{max(trq):5.1f} yaw={min(yr):5.3f}..{max(yr):5.3f}')
    print(f'      ctrlDC={min(dc):7.4f}..{max(dc):7.4f} max|DC|={max(map(abs,dc)):7.4f} desAngle={min(dsa):6.1f}..{max(dsa):6.1f} modelDC={min(mdc):7.4f}..{max(mdc):7.4f} big={max(big)}')
    peak = max(rows, key=lambda x: abs(x[2]))
    i = car.index(peak)
    print(f'      peak t={(peak[0]-car[0][0])/1e9:7.1f}s v={peak[1]:4.1f} angle={peak[2]:6.1f} torque={peak[3]:5.1f}')
    step = max(1, len(rows)//12)
    for x in rows[::step]:
        c = nearest(ct, ctrl, x[0]); m = nearest(mt, mdl, x[0])
        print(f'        t={(x[0]-car[0][0])/1e9:7.1f} v={x[1]:4.1f} ang={x[2]:6.1f} rate={x[4]:5.3f} DC={c[1]:7.4f} mDC={m[1]:7.4f}')

side = None; start = None; last = None
for row in car:
    t, left, right = row[0], row[5], row[6]
    cur = 'left' if left and not right else 'right' if right and not left else None
    if cur != side:
        if side is not None and (t-start) >= 300_000_000:
            describe(start, last, side)
        side, start = cur, t
    if side is not None:
        last = t
if side is not None and (last-start) >= 300_000_000:
    describe(start, last, side)
print('segments', paths)
