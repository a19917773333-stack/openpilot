# October 17 lean upstream sync

Source branch: `onemiless/openpilot:dev-sp-chipmunk`, original
`e0814246c918439f8d05d4518efaa649753e3f3b`. Upstream reviewed through
`lochuan/lean-mici:lean-master@be16822e572ce8c6e7def7cfd48d21eae7ffeef1`.

## Adopted

- Panda shutdown/child-signal handling and C4 reset enumeration wait, preserving
  C3 USB/F4, C3XL SPI and internal serial selection (`55826b38f3bf`).
- Torque-point cache serialization/writing at normal scheduling priority every
  60 seconds, with scheduling restoration on errors (`8f99f94c89e1`).
- Fixed EOF+60 ms reply cutoff, measured EOF-to-publication frame delay and
  existing fallback/blending (`7da0988df8d2`). OS04C10 readout is corrected to
  32.5 ms only; AR/OX sensor timing is unchanged (`cf4cc1943103`).
- CameraOffset/horizon shear in Python modeld (`9bec89383bed`), with the same
  per-calibration filter and physical intrinsics in native bigmodeld. Host
  parity supplies identical calibration sequences; asynchronous device
  delivery and moving calibration remain unqualified.
- Independent `/data/build` source and pinned submodule checkout, automatic
  cereal/model build, serial EKF compilation then parallel remaining targets,
  8 GB SCons cache (`1341d5782b61`, `668c27c93222`, adapted). Existing hardware,
  schema, params and ELF guards are retained; all DOS/H7 and legacy MPC
  artifacts plus every model chunk are required before publishing.

## User constraints and deferred work

The user explicitly excludes updater and installation-connectivity fixes.
The updater binary, updater/setup Python files, launch environment and all
AGNOS manifests remain byte-identical to the original source commit.

ABL and boot entries for C3/C3XL match `onemiless/openpilot:dev-sp` at remote
`f400fc5e56839c3ece7f999d18bd54842900df21`, including URL, hashes, sizes and slot
policy. The local dev-sp tip was advancing concurrently; its boot entries also
matched. C4 retains its own hardware-specific manifest. No partition is read
or written by this work, and AGNOS stays 19.6.27.

The proposed new-domain 19.6.27 system image returned HTTP 403 for HEAD and 404
for a ranged GET. Its URL was therefore not adopted. Toyota diagnostic/CAN
ownership features, Gitee mirroring, default never-power-off, and AGNOS 19.6.28
are excluded. No submodule pin changes.

## Evidence and scope

Artifacts: `/Users/mile/work/tesla-port/upstream-sync-20261017`.
The final `RESULT.json` identifies the exact source commit, verification scope,
unchanged-file hashes and commit list; `SHA256SUMS.json` covers the evidence.

Host evidence includes:

- Runtime entrypoint replay: 7 scenarios, including injected cache-write error.
- Reply/parser/blender/action/modelV2 serialization replay: 8 scenarios.
- Python/native geometry, actual padded-NV12 LUT and BGM1 header: 18 cases.
- Clean-build Git/submodule workflow and injected artifact failures: 7 cases;
  release profile matrix: 18 cases; service cleanup: 2 cases; failed checkout
  boundary: 4 cases; inherited dependency paths: 3 cases.
- Boot entry equality: 4 cases; 8 excluded files remain byte-identical.
- Existing model/Panda regressions: 24 passed; release regressions: 84 passed;
  native bigmodeld: 660 checks, zero failures; WarpLut/golden regressions pass.

385 historical one-second samples of accepted replies remain below 60 ms
(maximum 59.8772 ms). These omit unsampled and rejected replies and do not
measure the new branch's all-frame adoption rate or performance. Native
geometry code was compiled on the host. Device-only main.cc, camera timing,
the full ARM release and actual phone inference still require device checks.
No device build, installation, reboot, firmware flash or road test was run.

## Repeat host checks

From `/Users/mile/work/tesla-port/lean-mici`:

```bash
export PYTHONPATH="$PWD:$PWD/opendbc_repo:$PWD/msgq_repo:$PWD/rednose_repo:$PWD/tinygrad_repo"
PY=/Users/mile/Desktop/mo-op/.venv/bin/python
OUT=/Users/mile/work/tesla-port/upstream-sync-20261017
"$PY" tools/tesla/runtime_upstream_e2e.py --output "$OUT/runtime-after.json"
"$PY" tools/tesla/model_timing_e2e.py --output "$OUT/model-after.json" \
  --historical /Users/mile/work/tesla-port/c3xl-1344-20261010/phone-final/result.json \
  --historical /Users/mile/work/tesla-port/phone-onroad-20261010-5min/result.json
"$PY" tools/tesla/camera_offset_pipeline_e2e.py --output "$OUT/camera-after.json"
"$PY" tools/tesla/release_build_tree_e2e.py --output "$OUT/release-after.json"
"$PY" tools/tesla/bootchain_parity_e2e.py --source-repo /Users/mile/Desktop/mo-op \
  --source-ref f400fc5e56839c3ece7f999d18bd54842900df21 --output "$OUT/bootchain-parity.json"
```

Runtime/model/release changes are separate commits. Revert them in reverse
order to restore the original source. No persistent parameter migration or
device rollback is involved in these host-only changes.
