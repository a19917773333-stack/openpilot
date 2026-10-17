#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

# lean build: no driver monitoring, cabin camera unused
export DISABLE_DRIVER=1

export STAGING_ROOT="/data/safe_staging"

# A tici device-tree name cannot distinguish C3 from C3XL.
model_file="${SUNNYPILOT_HARDWARE_MODEL_FILE:-/sys/firmware/devicetree/base/model}"
profile_file="${SUNNYPILOT_HARDWARE_PROFILE_FILE:-/data/hardware_profile}"
profile_value="${SUNNYPILOT_HARDWARE_PROFILE:-}"
[ -z "$profile_value" ] && [ -f "$profile_file" ] && profile_value="$(cat "$profile_file")"
profile_value="${profile_value#"${profile_value%%[![:space:]]*}"}"
profile_value="${profile_value%"${profile_value##*[![:space:]]}"}"
device_model="$(tr -d '\0' < "$model_file" 2>/dev/null)"
export AGNOS_MANIFEST_FILE="openpilot/common/hardware/comma/agnos.json"
export AGNOS_SKIP_UPDATE=0
case "$device_model" in
  *tici)
    case "$profile_value" in
      c3|c3xl)
        AGNOS_MANIFEST_FILE="openpilot/common/hardware/comma/agnos-$profile_value.json"
        export AGNOS_VERSION="${AGNOS_VERSION:-19.7}"
        ;;
      *) AGNOS_SKIP_UPDATE=1 ;;
    esac ;;
esac
export AGNOS_MANIFEST_FILE AGNOS_SKIP_UPDATE

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.6.27"
fi

if [ -z "$AGNOS_ACCEPTED_VERSIONS" ]; then
  export AGNOS_ACCEPTED_VERSIONS="$AGNOS_VERSION"
fi

# Match the validated IFE road output and its model/UI intrinsics on C3XL.
if [ "$profile_value" = "c3xl" ]; then
  export C3XL_IFE_ROAD_SIZE="${C3XL_IFE_ROAD_SIZE:-1344x760}"
fi
