import sys, glob, os
from openpilot.tools.lib.logreader import LogReader
for p in sys.argv[1:]:
  first=last=None; n=0; kinds={}
  for m in LogReader(p):
    t=m.logMonoTime
    first=t if first is None else min(first,t); last=t if last is None else max(last,t)
    n+=1; w=m.which(); kinds[w]=kinds.get(w,0)+1
  print(os.path.basename(os.path.dirname(p)), 'n', n, 'first', first/1e9 if first else None, 'last', last/1e9 if last else None, 'dur', (last-first)/1e9 if first else None, 'top', sorted(kinds.items(), key=lambda kv:-kv[1])[:8])
