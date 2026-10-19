import sys
from openpilot.tools.lib.logreader import LogReader

path = sys.argv[1]
first = {}
for msg in LogReader(path):
    first.setdefault(msg.which(), msg)
for name in ('carState', 'controlsState', 'carControl', 'selfdriveState', 'drivingModelData', 'longitudinalPlan'):
    if name in first:
        print('\nFIRST', name)
        d = first[name].to_dict()[name]
        print('keys:', list(d.keys()))
        print(d)
        if name == 'carControl':
            break
