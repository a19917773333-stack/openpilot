# October 17 upstream sync: failures and acceptance before implementation

Target: `dev-sp-chipmunk`, starting at `e0814246c918439f8d05d4518efaa649753e3f3b`.
Upstream source: `lochuan/lean-mici:lean-master@be16822e572ce8c6e7def7cfd48d21eae7ffeef1`.

The user excludes updater and installation-connectivity changes. ABL and boot
must match the user's existing SP source; that source branch must be resolved
before changing any system manifest. No device operation or deployment is part
of these host checks.

Failure inventory, recorded before implementation:

- Panda: SIGINT during enumeration or firmware flashing must not start another
  child; SIGINT racing with Popen must reach that child. Preserve the selected
  internal serial, C3 USB/F4, C3XL SPI/type handling and existing bounded startup.
- Torque cache: serializing/writing learning points at FIFO priority can starve
  sensors. Cache every 60 seconds, drop realtime for serialization and writing,
  restore scheduling even if the write fails, and propagate that failure.
- Model timing: stale replies must never become the current frame; EOF and
  arrival must share BOOTTIME. Test disabled/warmup/disconnected/late/zero/reset
  paths, the 60 ms boundary, blending and published control metadata. Historical
  one-second samples are partial replay evidence, not all-frame or road proof.
- Camera offset: small/native big warps must use the same horizon, height,
  offset and per-axis physical intrinsics. Test zero/nonzero offsets, both
  cameras, AR/OX native/resized and C4 OS, padded NV12 and wire matrices.
- Release: a clean build must use our source repository and submodule pins;
  preserve profile checks, C3XL geometry, DOS/H7 firmware, both legacy MPC
  solver sets, schema/params and runtime artifact gates. Never build or clean
  the active runtime tree. Failure must reject publishing and restore service.
- Boot chain: compare ABL/boot entries byte-for-byte with the chosen SP source;
  retain per-profile selection, image hashes, sizes and slot policy. A URL-only
  system relocation must not change an image hash or bump AGNOS.

Forward path: separate scoped commits for runtime fixes, model geometry/timing,
release and manifests. Rollback: revert the corresponding commits in reverse
order; no parameter migration or device rollback is needed for host-only work.
Repeatable host pipeline scripts and JSON/hash artifacts accompany the changes.

## Follow-up: use the dev-sp system image too

The user now requires C3/C3XL `system` to match `onemiless/openpilot:dev-sp`,
in addition to ABL/boot. Source was refreshed at
`f400fc5e56839c3ece7f999d18bd54842900df21`: AGNOS 19.7. This supersedes the
earlier decision to retain lean's 19.6.27 system for those profiles.

Failures to reject before implementation:

- Copying only the URL while retaining lean's sparse/raw hashes, verification
  marker, size or alternate-image fields creates an inconsistent manifest.
- Keeping 19.6.27 in the launcher's accepted-version default skips the requested
  system update; the target and accepted defaults must both become 19.7.
- A global version bump would make C4's unchanged manifest disagree with its
  required version. Resolve defaults through the existing hardware/profile gate.
- Regenerating manifests must reproduce the full selected SP entries and keep
  ABL/boot unchanged; updater/setup code remains excluded.

Acceptance is an offline manifest-generation and launch-selection pipeline with
JSON evidence, including source equality and version/profile selection. No
device flashing or image execution is included. Roll back with one scoped revert.
