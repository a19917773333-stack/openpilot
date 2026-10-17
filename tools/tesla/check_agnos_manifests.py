#!/usr/bin/env python3
"""Offline full-chain selection and rejection check; writes only stdout."""
import copy
import json
import subprocess
from build_agnos_manifest import CHAIN, OUT, REF, build, load
from openpilot.sunnypilot.hardware.agnos import UnsafeBootChainManifest, validate_agnos_manifest
from openpilot.sunnypilot.hardware.profile import HardwareProfile

lean = load(REF / "lean-c4.json")
for profile in ("c3", "c3xl"):
  parts = json.loads((OUT / f"agnos-{profile}.json").read_text())
  assert parts == build(profile)
  by_name = {p["name"]: p for p in parts}
  assert by_name["system"] == load(REF / f"devsp-{profile}.json")["system"]
  assert by_name["boot"]["hash"] != lean["boot"]["hash"]
  validate_agnos_manifest(parts, HardwareProfile(profile))
  for name in CHAIN:
    assert by_name[name]["has_ab"] is True and by_name[name]["full_check"] is True
    bad = copy.deepcopy(parts)
    next(p for p in bad if p["name"] == name)["hash"] = "0" * 64
    try:
      validate_agnos_manifest(bad, HardwareProfile(profile))
    except UnsafeBootChainManifest:
      pass
    else:
      raise AssertionError(f"{profile}/{name}: corrupted hash accepted")
    validate_agnos_manifest(bad, HardwareProfile.STANDARD)
  if profile == "c3xl":
    assert by_name["abl"]["hash"] != lean["abl"]["hash"]
  print(f'{profile}: chain verified; abl equals C4: {by_name["abl"]["hash"] == lean["abl"]["hash"]}')
original = subprocess.check_output(["git", "show", "4802cb2fe8c993fd7852b1f78be5b8b9604dd5a5:openpilot/common/hardware/comma/agnos.json"])
assert (REF / "lean-c4.json").read_bytes() == (OUT / "agnos.json").read_bytes() == original
print("C4 manifest byte-exact; corrupted chains rejected")
