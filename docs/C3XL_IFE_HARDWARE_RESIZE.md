# C3XL IFE hardware road-camera resize

The C3XL OX03C10 road-camera path defaults to 1344×760 IFE output before VisionIPC/model upload when started through the normal launcher. This supplies the input size required by the CTM artifact without a device-local activation export. It avoids the extra GPU resize, synchronization and copies from the software-resize experiment. Model weights, frame cadence, control permissions and frame-drop checks are unchanged.

## Activation and scope

`launch_env.sh` exports `C3XL_IFE_ROAD_SIZE=1344x760` by default for the C3XL hardware profile; keep `C3XL_CTMV2_INPUT_RESIZE` unset. Existing explicit values, including an empty value, are preserved. Standard hardware does not receive this default. Direct process launches that bypass `launch_env.sh` must supply the resize environment explicitly.

The camera override requires a C3XL build profile, OX03C10 at1928×1208, an IFE-processed stream and road camera0/1. Camera intrinsics use the existing hardware-profile resolver, matching the launcher and build, including recovery installs that infer C3XL from `comma tici` without a profile file. The cabin/BPS stream and sensor mode remain unchanged. Other combinations retain their original output dimensions. The selector refuses startup when the requested IFE dimensions disagree with actual road streams.

IFE Y/UV dimensions, phase steps and output buffers are configured together. Road-camera intrinsics use independent horizontal/vertical factors; exposure windows, padded row access and camera-offset shear receive matching corrections. An explicit native-resolution override retains the original behavior. This is global road-stream resizing: the UI, model and other consumers share the smaller road images; it is not an additional private stream.

## Evidence

The implementation was built from device source430fe0aff6da30c5ed7441979cd9c890b78b5c5f with tinygradf6fc4e3f2c3db5fae1e19cbfbc3ad9fc579a12ae. Both road cameras produced1344×760, stride1408, UV offset1081344, at approximately20Hz with no missing frames in the camera-only check. Parked before/after images showed the full field of view rather than a crop.

A short live-camera CTMv2 check (ref37bfa1413edcdc2e8844984b83727c33f81d8f46,100 measured frames after10 warmup frames) returned19.9897Hz, P50 processing40.3669ms, P9540.9535ms, max73.4875ms, one sample over50ms and no camera-frame gaps. The TSFM fallback model also ran at1344×760. This short run does not establish universal long-term performance or safety certification. The user subsequently confirmed the solution usable and requested local branch synchronization on2026-09-13.

Register definitions and formula reference: Qualcomm MNDS16 Titan17x code and Titan170 IFE register definitions in public source commit36fc163a534963a5b3af52186af5efcc63401ad2 of `comprehensive9/vendor_qcom_proprietary`. `ife_scale.h` implements the required formulas independently; vendor source is not included. Interpolation resolution occupies bits29:28, phase uses Q(14+resolution), and dimensions encode N−1.

## Rollback

Set `export C3XL_IFE_ROAD_SIZE=1928x1208` in the startup environment before launching openpilot and restart through the normal offroad lifecycle. This restores original road dimensions and geometry without deleting models or calibration; unsetting the variable now selects the 1344×760 default again. Use a model artifact that supports the selected dimensions. Historical device-specific rollback material is under `/data/c3xl-ife-deploy-20260913-v1/`; its old helper predates the new default and must not be assumed to disable it.

Only CTMv2 plus the selected TSFM fallback were checked in the historical device run above. Confirm other model artifacts provide the required resolution before using them with this default. C3XL fallback builds include both 1344×760 and 1928×1208. Hardware binaries, model files and private camera frames are not included in this source synchronization.
