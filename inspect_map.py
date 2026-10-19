import sys
from openpilot.tools.lib.logreader import LogReader
p=sys.argv[1]; first={}
for m in LogReader(p):
  if m.which()=='liveMapDataSP':
    d=m.to_dict()['liveMapDataSP']
    print(list(d.keys()))
    for k,v in d.items(): print(k,str(v)[:500])
    break
