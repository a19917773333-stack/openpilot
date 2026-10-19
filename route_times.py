import sys
from openpilot.tools.lib.logreader import LogReader
p=sys.argv[1]
first={}; last={}; n={}
for m in LogReader(p):
  w=m.which(); t=m.logMonoTime
  if w not in ('carState','controlsState','carControl','selfdriveState','drivingModelData'): continue
  first.setdefault(w,t); last[w]=t; n[w]=n.get(w,0)+1
for w in first: print(w, n[w], first[w]/1e9, last[w]/1e9)
