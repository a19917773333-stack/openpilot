import sys
from collections import Counter
from openpilot.tools.lib.logreader import LogReader

path = sys.argv[1]
counts = Counter()
first = {}
for msg in LogReader(path):
    which = msg.which()
    counts[which] += 1
    first.setdefault(which, msg)
print('COUNTS')
for k, v in sorted(counts.items()):
    print(k, v)
for name in ('carState', 'controlsState', 'selfdriveState', 'modelV2', 'modelDataV2SP', 'navLaneIntentSP', 'navAssistStateSP'):
    if name in first:
        print('\nFIRST', name)
        d = first[name].to_dict()
        if name == 'controlsState':
            keep = {k: d.get(k) for k in ('desiredCurvature', 'curvature', 'steeringAngleDeg', 'vEgo', 'active', 'enabled')}
        elif name == 'carState':
            keep = {k: d.get(k) for k in ('vEgo', 'steeringAngleDeg', 'steeringTorque', 'steeringPressed', 'leftBlinker', 'rightBlinker', 'leftBlindspot', 'rightBlindspot')}
        else:
            keep = list(d.keys())[:30]
        print(keep)
