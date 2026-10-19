import sys
from openpilot.tools.lib.logreader import LogReader
times=[]; seen=set()
for p in sys.argv[1:]:
  for m in LogReader(p):
    if m.which()=='cameraOdometry' and m.logMonoTime not in seen:
      seen.add(m.logMonoTime); times.append(m.logMonoTime)
times.sort(); t0=times[0]
print(len(times))
for t in times[:20]: print(round((t-t0)/1e9,2))
print('...')
for t in times[-20:]: print(round((t-t0)/1e9,2))
