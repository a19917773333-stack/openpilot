import pytest

from tools.release.check_model_artifacts import DM_WARP_ARTIFACTS, JETLINK_WARP_ARTIFACTS, validate_model_artifacts


@pytest.mark.parametrize("directory", ["sp", "openpilot"])
def test_requires_both_nonempty_dm_warps(tmp_path, directory):
  root = tmp_path / directory
  models = root / "openpilot/selfdrive/modeld/models"
  models.mkdir(parents=True)
  jetlink_models = root / "openpilot/sunnypilot/jetlink_adapter/models"
  jetlink_models.mkdir(parents=True)
  with pytest.raises(RuntimeError, match="DM warp"):
    validate_model_artifacts(root)
  for name in DM_WARP_ARTIFACTS:
    (models / name).write_bytes(b"compiled warp")
  with pytest.raises(RuntimeError, match="Jetlink warp"):
    validate_model_artifacts(root)
  for name in JETLINK_WARP_ARTIFACTS:
    (jetlink_models / name).write_bytes(b"compiled warp")
  validate_model_artifacts(root)
  (models / DM_WARP_ARTIFACTS[0]).write_bytes(b"")
  with pytest.raises(RuntimeError, match="1928x1208"):
    validate_model_artifacts(root)
  (models / DM_WARP_ARTIFACTS[0]).write_bytes(b"compiled warp")
  (jetlink_models / JETLINK_WARP_ARTIFACTS[1]).write_bytes(b"")
  with pytest.raises(RuntimeError, match="1344x760"):
    validate_model_artifacts(root)
