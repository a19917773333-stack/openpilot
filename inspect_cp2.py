import sys
from openpilot.tools.lib.logreader import LogReader
for m in LogReader(sys.argv[1]):
  if m.which()=='carParams':
    cp=m.carParams
    for k in ('carFingerprint','wheelbase','steerRatio','steerRatioSP','steerActuatorDelay','lateralTuning','mass','minSteerSpeed','maxSteer','maxSteerAngleDeg'):
      try: print(k, getattr(cp,k))
      except Exception as e: pass
    break
