import io
import os
import sys
import pickle
import struct
from pathlib import Path

from openpilot.common.file_chunker import get_chunk_name, get_manifest_path, open_file_chunked
from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.usb import CHESTNUT_USB_PRODUCT, USB_DEVICES_PATH, is_chestnut_usb_id

MODELS_DIR = Path(__file__).resolve().parent / 'models'


def modeld_pkl_path(chestnut: bool):
  prefix = 'big_' if chestnut else ''
  return MODELS_DIR / f'{prefix}driving_tinygrad.pkl'

def load_oob(path, chestnut=False):
  from tinygrad import Context, Device
  from tinygrad.device import Buffer
  device = 'USB+AMD:LLVM' if chestnut else 'QCOM' if AGNOS else 'METAL' if sys.platform == 'darwin' else 'CPU:LLVM'
  chunks = 0 if os.path.isfile(path) else int(Path(get_manifest_path(path)).read_text())
  paths = [get_chunk_name(path, i, chunks) for i in range(chunks)] or [path]
  with Context(DEV=device), open_file_chunked(path) as f:
    opcodes = f.read(struct.unpack('<q', f.read(8))[0])
    buffers, stage = Buffer(Device.DEFAULT, sum(map(os.path.getsize, paths)) - 8 - len(opcodes)).ensure_allocated(), bytearray(64 << 20)
    for offset in range(0, buffers.nbytes, len(stage)):
      n = f.readinto(piece := memoryview(stage)[:min(len(stage), buffers.nbytes - offset)])
      if n != len(piece):
        raise EOFError("incomplete model buffer")
      buffers.view(n, offset).ensure_allocated().copy_from(Buffer("PYTHON", n, initial_value=piece[:n]))
    return type("_Unpickler", (pickle.Unpickler,), {"persistent_load": lambda self, pid: buffers.view(*pid)})(io.BytesIO(opcodes)).load()

def chestnut_present() -> bool:
  for d in USB_DEVICES_PATH.glob("*"):
    try:
      usb_id = (int((d / "idVendor").read_text(), 16), int((d / "idProduct").read_text(), 16))
      product = (d / "product").read_text().strip()
      if is_chestnut_usb_id(*usb_id) and product == CHESTNUT_USB_PRODUCT:
        return True
    except Exception:
      pass
  return False

def _compiled_file(path: Path) -> bool:
  try:
    with path.open('rb') as f:
      header = f.read(64)
      return bool(header) and not header.startswith(b'version https://git-lfs.github.com/spec/v1')
  except OSError:
    return False


def compiled_model_file(path: Path, expected_chunks: int | None = None) -> bool:
  """Cheap loader-file completeness check; download/activation verifies hashes."""
  if path.is_file():
    return _compiled_file(path)
  try:
    count = int(Path(get_manifest_path(path)).read_text().strip())
    # ponytail: cap probes at 1024 chunks (~45 GiB); raise only for larger supported models.
    return 0 < count <= 1024 and (expected_chunks is None or count == expected_chunks) and all(
      _compiled_file(Path(get_chunk_name(path, i, count))) for i in range(count))
  except (OSError, ValueError):
    return False


def chestnut_warps_compiled() -> bool:
  return all(_compiled_file(MODELS_DIR / f'big_driving_warp_{size}_tinygrad.pkl') for size in ('1344x760', '1928x1208'))


def chestnut_compiled() -> bool:
  return compiled_model_file(modeld_pkl_path(chestnut=True)) and chestnut_warps_compiled()
