#!/usr/bin/env python3
"""Keep dev-sp boot chains and system byte-equivalent on C3 and C3XL."""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
REF = ROOT / "docs/tesla/ref/agnos"
OUT = ROOT / "openpilot/common/hardware/comma"
CHAIN = ("xbl", "xbl_config", "abl", "aop", "devcfg", "boot")


def load(path):
  return {part["name"]: part for part in json.loads(path.read_text())}


def build(profile):
  devsp = load(REF / f"devsp-{profile}.json")
  return [devsp[name] for name in (*CHAIN, "system")]


if __name__ == "__main__":
  for profile in ("c3", "c3xl"):
    path, manifest = OUT / f"agnos-{profile}.json", build(profile)
    if "--check" in sys.argv:
      assert json.loads(path.read_text()) == manifest, f"{path} drifted"
    else:
      path.write_text(json.dumps(manifest, indent=2) + "\n")
  print("ok")
