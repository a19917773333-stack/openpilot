import sys
from openpilot.tools.lib.logreader import LogReader
p=sys.argv[1]
for m in LogReader(p):
  if m.which()=='cameraOdometry':
    d=m.to_dict()['cameraOdometry']
    print(list(d.keys()))
    for k,v in d.items(): print(k,str(v)[:400])
    break
