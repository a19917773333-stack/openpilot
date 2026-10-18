"""Reject prebuilt releases missing warp artifacts required at runtime."""
from pathlib import Path
import sys


DM_WARP_ARTIFACTS = (
  "dm_warp_1928x1208_tinygrad.pkl",
  "dm_warp_1344x760_tinygrad.pkl",
)

JETLINK_WARP_ARTIFACTS = (
  "warp_1928x1208_512x256_tinygrad.pkl",
  "warp_1344x760_512x256_tinygrad.pkl",
)


def _missing(models: Path, artifacts: tuple[str, ...]) -> list[str]:
  return [name for name in artifacts if not (models / name).is_file() or (models / name).stat().st_size == 0]


def validate_model_artifacts(root: Path) -> None:
  dm_models = root / "openpilot/selfdrive/modeld/models"
  if missing := _missing(dm_models, DM_WARP_ARTIFACTS):
    raise RuntimeError(f"Prebuilt release is missing DM warp artifacts: {', '.join(missing)}")

  jetlink_models = root / "openpilot/sunnypilot/jetlink_adapter/models"
  if missing := _missing(jetlink_models, JETLINK_WARP_ARTIFACTS):
    raise RuntimeError(f"Prebuilt release is missing Jetlink warp artifacts: {', '.join(missing)}")


if __name__ == "__main__":
  validate_model_artifacts(Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2])
