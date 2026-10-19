import sys, collections
from openpilot.tools.lib.logreader import LogReader
for p in sys.argv[1:]:
  c=collections.Counter(m.which() for m in LogReader(p))
  print(p, c.most_common(35))
