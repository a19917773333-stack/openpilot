import sys
from openpilot.tools.lib.logreader import LogReader
path = sys.argv[1]
first = {}
for msg in LogReader(path):
    first.setdefault(msg.which(), msg)
for name in ('drivingModelData', 'liveMapDataSP', 'lateralDelay', 'vehicleParameters'):
    if name in first:
        print('\nFIRST', name)
        d = first[name].to_dict()[name]
        print('keys:', list(d.keys()))
        print(d)
