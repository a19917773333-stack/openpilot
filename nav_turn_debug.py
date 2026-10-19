import glob
import re
import sys

from openpilot.tools.lib.logreader import LogReader


def seg_key(path):
  m = re.search(r"--(\d+)/qlog", path)
  return int(m.group(1)) if m else 0


def nearest(arr, t, key="t"):
  if not arr:
    return {}
  times = [r[key] for r in arr]
  i = __import__("bisect").bisect_left(times, t)
  if i <= 0:
    return arr[0]
  if i >= len(arr):
    return arr[-1]
  return arr[i - 1] if t - times[i - 1] <= times[i] - t else arr[i]


route = sys.argv[1]
paths = sorted(glob.glob(route + "--*/qlog.zst"), key=seg_key)
data = {k: [] for k in ("car", "ctrl", "cc", "md", "mdsp", "nav", "sd")}

for path in paths:
  for msg in LogReader(path):
    t = msg.logMonoTime
    w = msg.which()
    if w == "carState":
      s = msg.carState
      data["car"].append({"t": t, "v": s.vEgo, "ang": s.steeringAngleDeg, "trq": s.steeringTorque,
                          "press": s.steeringPressed, "lb": s.leftBlinker, "rb": s.rightBlinker})
    elif w == "controlsState":
      s = msg.controlsState
      data["ctrl"].append({"t": t, "dc": s.desiredCurvature, "curv": s.curvature})
    elif w == "carControl":
      s = msg.carControl
      data["cc"].append({"t": t, "lat": s.latActive, "torque": s.actuators.torque,
                         "sang": s.actuators.steeringAngleDeg})
    elif w == "drivingModelData":
      s = msg.drivingModelData
      data["md"].append({"t": t, "mdc": s.action.desiredCurvature, "big": s.big})
    elif w == "modelDataV2SP":
      data["mdsp"].append({"t": t, "turn": int(msg.modelDataV2SP.laneTurnDirection)})
    elif w == "navLaneIntentSP":
      s = msg.navLaneIntentSP
      data["nav"].append({"t": t, "valid": s.valid, "sig": s.signalRequested,
                          "dir": int(s.direction), "reason": s.reason})
    elif w == "selfdriveState":
      s = msg.selfdriveState
      data["sd"].append({"t": t, "active": s.active, "enabled": s.enabled,
                         "alert": s.alertText1})

for k in data:
  data[k].sort(key=lambda r: r["t"])

car = data["car"]
if not car:
  raise SystemExit("no carState")
t0 = car[0]["t"]

active = []
for r in car:
  turn = nearest(data["mdsp"], r["t"]).get("turn", 0)
  nav = nearest(data["nav"], r["t"])
  nav_on = bool(nav.get("valid") and nav.get("sig"))
  lamp = bool(r["lb"] or r["rb"])
  active.append(turn != 0 or nav_on or lamp)

intervals = []
start = None
for r, on in zip(car, active, strict=True):
  if on and start is None:
    start = r["t"]
  elif not on and start is not None:
    intervals.append((start, prev))
    start = None
  prev = r["t"]
if start is not None:
  intervals.append((start, prev))

for a, b in intervals:
  if b - a < 0.5e9:
    continue
  rows = [r for r in car if a <= r["t"] <= b]
  if not rows:
    continue
  turns = [nearest(data["mdsp"], r["t"]).get("turn", 0) for r in rows]
  navs = [nearest(data["nav"], r["t"]) for r in rows]
  print(f"\n=== {((a - t0) / 1e9):7.1f}-{((b - t0) / 1e9):7.1f}s dur={(b-a)/1e9:4.1f}s "
        f"turn={max(turns)} navdir={max(n.get('dir', 0) for n in navs)} ===")
  step = max(1, len(rows) // 24)
  for r in rows[::step]:
    c = nearest(data["ctrl"], r["t"])
    cc = nearest(data["cc"], r["t"])
    m = nearest(data["md"], r["t"])
    n = nearest(data["nav"], r["t"])
    sd = nearest(data["sd"], r["t"])
    print(f"t={((r['t']-t0)/1e9):7.1f} v={r['v']:5.2f} ang={r['ang']:7.1f} press={int(r['press'])} "
          f"L={int(r['lb'])}R={int(r['rb'])} lat={int(cc.get('lat',0))} act={int(sd.get('active',0))} "
          f"turn={nearest(data['mdsp'], r['t']).get('turn',0)} nav={int(n.get('sig',0))}/{n.get('dir',0)} "
          f"DC={c.get('dc',0):8.4f} mDC={m.get('mdc',0):8.4f} curv={c.get('curv',0):8.4f} "
          f"big={int(m.get('big',0))} alert={sd.get('alert','')}")
