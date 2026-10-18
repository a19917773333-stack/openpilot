"""Read-only eGPU status model and the small left-side HUD panel."""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path

import pyray as rl

from openpilot.common.hardware.usb import CHESTNUT_USB_IDS

# The owner keeps what the phone said at hello and the cable's negotiated
# speed here; the bottom strip reads them, never the link itself.
JETLINK_STATUS_RECORD = Path('/dev/shm/jetlink/status.json')
JETLINK_SERVER_RECORD = Path('/dev/shm/jetlink/server.json')
JETLINK_RECORD_PERIOD = 1.0


@dataclass(frozen=True)
class EgpuStatus:
  visible: bool
  healthy: bool
  headline: str
  details: tuple[str, ...]


@dataclass(frozen=True)
class CompactEgpuStatus:
  visible: bool
  healthy: bool
  text: str


@dataclass(frozen=True)
class EgpuPanelStyle:
  font_size: int
  line_height: int
  panel_width: int
  bottom_gap: int
  background_alpha: int


@dataclass(frozen=True)
class EgpuSidebarStatus:
  value: str
  severity: str
  detail: str


@dataclass(frozen=True)
class JetlinkGpuStatus:
  """The accelerator tile while a phone, not a Chestnut, runs the model."""
  value: str
  severity: str
  detail: str


def build_jetlink_gpu_status(*, state: str, present: bool, progress: dict | None = None,
                             frame_ms: float = 0.0, model: str = "") -> JetlinkGpuStatus:
  """Map jetlink's accelerator state to the sidebar's GPU tile.

  `state` is the ChestnutState name the UI already derives from
  jetlink.openpilot.status.Status.icon, so the tile and the chestnut icon agree.
  The phone is this device's GPU: the tile appears as soon as the link is
  enabled, gray when nothing is attached, green only once frames come back.
  """
  if state == 'failed':
    return JetlinkGpuStatus('ERR', 'danger', '手机大模型加载或运行失败，已回退小模型')
  if state == 'disconnected' or (not present and state != 'loading'):
    return JetlinkGpuStatus('OFFLINE', 'disabled', '未检测到手机（USB 未连接）')
  if state == 'uncompiled':
    return JetlinkGpuStatus('NO MODEL', 'warning', '手机大模型尚未就绪')
  if state == 'loading':
    frac = float((progress or {}).get('frac') or 0.0)
    stage = str((progress or {}).get('stage') or '')
    if stage == 'build' and 0.0 < frac < 1.0:
      return JetlinkGpuStatus(f'LOAD {int(round(frac * 100))}%', 'progress', '手机正在编译大模型')
    return JetlinkGpuStatus('WAIT', 'progress', '手机已连接，正在加入大模型')
  if state in ('active', 'ready', 'waiting'):
    detail = '手机大模型运行中' if state == 'active' else '手机大模型已就绪，等待可切换的时机'
    if state == 'active' and frame_ms > 0:
      detail += f' · {frame_ms:.0f} ms/帧'
    if model:
      detail += f' · {model}'
    return JetlinkGpuStatus('ACTIVE' if state == 'active' else 'READY', 'good', detail)
  return JetlinkGpuStatus('WAIT', 'warning', '等待手机')


def jetlink_gpu_icon_severity(*, state: str, present: bool) -> str:
  """The onroad GPU icon's colour: green only while the phone drives."""
  if not present:
    return 'disabled'
  if state == 'failed':
    return 'danger'
  if state == 'active':
    return 'good'
  return 'warning'


@dataclass(frozen=True)
class JetlinkHardware:
  device: str = ''
  backend: str = ''
  usb_speed: str = ''


def _short_device(device: str) -> str:
  # "htp-whole-SM8750" is the NPU's name for the phone; the model is the part
  # a driver can tell apart
  return device.rsplit('-', 1)[-1] if device else ''


def _short_usb_speed(speed: str) -> str:
  return {'super-speed': 'USB5', 'high-speed': 'USB2', 'full-speed': 'USB1', 'low-speed': 'USB1'}.get(speed, '')


_hardware_cache: tuple[float, JetlinkHardware] = (0.0, JetlinkHardware())
_hardware_loaded = False


def read_jetlink_hardware(now: float | None = None, period: float = JETLINK_RECORD_PERIOD) -> JetlinkHardware:
  """The phone's own name and the cable speed, from the owner's records.

  The bottom strip draws at 60 Hz; these are two small files rewritten on a
  state change, so they are read at most once a second. A missing or
  half-written record keeps the last answer rather than blanking the line.
  """
  global _hardware_cache, _hardware_loaded
  at, cached = _hardware_cache
  now = time.monotonic() if now is None else now
  if _hardware_loaded and now - at < period:
    return cached

  device, backend = cached.device, cached.backend
  speed = cached.usb_speed
  try:
    server = json.loads(JETLINK_SERVER_RECORD.read_text())
    device = _short_device(str(server.get('device') or ''))
    backend = str(server.get('backend') or '').upper()
  except Exception:
    pass
  try:
    record = json.loads(JETLINK_STATUS_RECORD.read_text())
    speed = _short_usb_speed(str(record.get('speed') or ''))
  except Exception:
    pass

  cached = JetlinkHardware(device, backend, speed)
  _hardware_cache = (now, cached)
  _hardware_loaded = True
  return cached


@dataclass(frozen=True)
class JetlinkCompactStatus:
  visible: bool
  text: str
  severity: str


def build_compact_jetlink_status(*, state: str, present: bool, progress: dict | None = None,
                                 model_fps: float = 0.0, frame_ms: float = 0.0,
                                 device: str = "", backend: str = "",
                                 usb_speed: str = "") -> JetlinkCompactStatus:
  """The bottom strip's JetLink line, worded like the eGPU one beside it.

  `state` is the ChestnutState name the UI derives from jetlink's own status,
  so a phone and a Chestnut are described by the same vocabulary.
  """
  if not present:
    return JetlinkCompactStatus(False, '', 'disabled')
  if state == 'failed':
    return JetlinkCompactStatus(True, 'JetLink: ERR', 'danger')
  if state == 'uncompiled':
    return JetlinkCompactStatus(True, 'JetLink: NO MODEL', 'warning')
  if state == 'loading':
    stage = str((progress or {}).get('stage') or '')
    frac = float((progress or {}).get('frac') or 0.0)
    if 0.0 < frac < 1.0 and stage == 'build':
      return JetlinkCompactStatus(True, f'JetLink: LOAD {int(round(frac * 100))}%', 'progress')
    if 0.0 < frac < 1.0 and stage == 'upload':
      return JetlinkCompactStatus(True, f'JetLink: UPLOAD {int(round(frac * 100))}%', 'progress')
    return JetlinkCompactStatus(True, 'JetLink: JOIN', 'progress')
  if state in ('ready', 'waiting'):
    return JetlinkCompactStatus(True, 'JetLink: READY', 'good')
  if state == 'active':
    parts = ['JetLink:']
    if model_fps > 0:
      parts.append(f'{model_fps:.1f}FPS')
    if frame_ms > 0:
      parts.append(f'{frame_ms:.0f}ms')
    parts.extend(extra for extra in (usb_speed, device, backend) if extra)
    return JetlinkCompactStatus(True, ' '.join(parts), 'good')
  return JetlinkCompactStatus(True, 'JetLink: WAIT', 'progress')


def build_egpu_sidebar_status(*, present: bool, compiled: bool, link_state: str | None,
                              usb_speed_mbps: int, pcie_ltssm: int | None,
                              eject_status: str | None, loading: bool,
                              active: bool | None, loading_progress: int = 0,
                              model_failed: bool = False) -> EgpuSidebarStatus:
  if eject_status == "ejecting":
    return EgpuSidebarStatus("REMOVE...", "progress", "正在安全卸载 eGPU")
  if eject_status == "safe":
    return EgpuSidebarStatus("REMOVE", "good", "已安全卸载，可以断开 eGPU")
  if eject_status == "error":
    return EgpuSidebarStatus("EJECT ERR", "danger", "eGPU 安全卸载失败")
  if not present:
    return EgpuSidebarStatus("OFFLINE", "disabled", "未检测到 eGPU USB 设备")
  if link_state == "usb_degraded" or 0 < usb_speed_mbps < 5000:
    return EgpuSidebarStatus(f"USB {usb_speed_mbps}", "danger", f"USB 链路低于 5000 Mbps（当前 {usb_speed_mbps} Mbps）")
  if link_state == "pcie_down":
    ltssm = f"0x{pcie_ltssm:02X}" if pcie_ltssm is not None else "未知"
    return EgpuSidebarStatus("PCIE ERR", "danger", f"USB 正常，但 PCIe 未进入 L0（LTSSM {ltssm}）")
  if link_state == "check_error":
    return EgpuSidebarStatus("LINK ERR", "danger", "无法被动读取 PCIe 链路状态")
  if link_state != "ready":
    return EgpuSidebarStatus("CHECKING", "warning", f"USB {usb_speed_mbps or '?'} Mbps，正在确认 PCIe 状态")
  if not compiled:
    return EgpuSidebarStatus("NO MODEL", "warning", "USB/PCIe 正常，但默认大模型尚未编译")
  if loading:
    return EgpuSidebarStatus(f"LOAD {loading_progress}%", "progress", f"USB/PCIe 正常，大模型加载 {loading_progress}%")
  if model_failed or active is False:
    return EgpuSidebarStatus("MODEL ERR", "danger", "链路正常，但默认大模型加载或运行失败")
  if active is True:
    return EgpuSidebarStatus("ACTIVE", "good", f"eGPU 大模型运行中 · USB {usb_speed_mbps} Mbps · PCIe L0")
  return EgpuSidebarStatus("READY", "good", f"eGPU 已就绪 · USB {usb_speed_mbps} Mbps · PCIe L0")


def classify_egpu_link_state(*, present: bool, usb_speed_mbps: int, telemetry_alive: bool,
                             telemetry_valid: bool, pcie_ltssm: int) -> str:
  if not present:
    return "disconnected"
  if usb_speed_mbps < 5000:
    return "usb_degraded"
  if not telemetry_alive:
    return "unchecked"
  if not telemetry_valid:
    return "check_error"
  if pcie_ltssm != 0x78:
    return "pcie_down"
  return "ready"


def egpu_icon_visible(*, connected: bool) -> bool:
  """The onroad source icon is persistent for exactly the USB-connected state."""
  return connected


def resolve_egpu_connection(device_state) -> bool:
  """Return the current physical Chestnut USB presence without trip latching."""
  return bool(device_state.chestnutPresent)


def egpu_panel_style(*, compact: bool) -> EgpuPanelStyle:
  return EgpuPanelStyle(
    font_size=48,
    line_height=58,
    panel_width=1200,
    bottom_gap=73 if compact else 293,
    background_alpha=0,
  )


def build_egpu_status(*, connected: bool, compiled: bool, loading: bool, active: bool | None,
                      model_alive: bool, model_big: bool, telemetry_valid: bool,
                      model_name: str = "",
                      loading_progress: int = 0,
                      usb_speed_mbps: int = 0, model_fps: float = 0.0,
                      power_w: float = 0.0, temp_c: float = 0.0, memory_temp_c: float = 0.0,
                      memory_used_mb: int = 0, memory_total_mb: int = 0,
                      gpu_usage_percent: int = 0, gpu_clock_mhz: int = 0,
                      fan_speed_rpm: int = 0) -> EgpuStatus:
  if not connected:
    return EgpuStatus(False, False, "", ())

  model_label = model_name.strip() or "大模型"
  if not compiled:
    return EgpuStatus(True, False, f"{model_label} · 大模型未编译", ())
  if loading:
    return EgpuStatus(True, False, f"{model_label} · 正在加载大模型 · {loading_progress}%", ())
  if active is False:
    return EgpuStatus(True, False, f"{model_label} · 大模型失败 · 已回退小模型", ())
  if active is not True:
    return EgpuStatus(True, False, f"{model_label} · 等待模型启动", ())
  if not model_alive or not model_big:
    return EgpuStatus(True, False, f"{model_label} · 模型流中断 · 已回退/等待", ())
  if not telemetry_valid:
    return EgpuStatus(True, False, f"{model_label} · 大模型运行中 · 遥测暂不可用", ())

  used_gb = memory_used_mb / 1024.0
  total_gb = memory_total_mb / 1024.0
  degraded_link = 0 < usb_speed_mbps < 5000
  headline = (f"{model_label} · 大模型运行中 · USB 链路降速 · {model_fps:.1f} FPS" if degraded_link else
              f"{model_label} · 大模型运行中 · {model_fps:.1f} FPS")
  return EgpuStatus(True, not degraded_link, headline, (
    f"功耗 {power_w:.0f} W",
    f"GPU {temp_c:.0f}°C · 显存 {memory_temp_c:.0f}°C · {used_gb:.1f}/{total_gb:.1f} GB",
    f"负载 {gpu_usage_percent}% · {gpu_clock_mhz} MHz · 风扇 {fan_speed_rpm} RPM",
  ))


def build_compact_egpu_status(*, connected: bool, compiled: bool, loading: bool, active: bool | None,
                              model_alive: bool, model_big: bool, telemetry_valid: bool,
                              model_name: str = "", loading_progress: int = 0,
                              model_fps: float = 0.0, power_w: float = 0.0,
                              temp_c: float = 0.0, memory_temp_c: float = 0.0,
                              memory_used_mb: int = 0, memory_total_mb: int = 0,
                              gpu_usage_percent: int = 0) -> CompactEgpuStatus:
  """Compact model/GPU status for the left side of the bottom onroad strip."""
  if not connected:
    return CompactEgpuStatus(False, False, "")

  model_label = model_name.strip() or "MODEL"
  if not compiled:
    return CompactEgpuStatus(True, False, f"{model_label}: NO MODEL")
  if loading:
    return CompactEgpuStatus(True, False, f"{model_label}: LOAD {loading_progress}%")
  if active is False:
    return CompactEgpuStatus(True, False, f"{model_label}: ERR")
  if active is not True:
    return CompactEgpuStatus(True, False, f"{model_label}: WAIT")
  if not model_alive or not model_big:
    return CompactEgpuStatus(True, False, f"{model_label}: STREAM ERR")
  if not telemetry_valid:
    return CompactEgpuStatus(True, False, f"{model_label}: RUN")

  used_gb = memory_used_mb / 1024.0
  total_gb = memory_total_mb / 1024.0
  text = "".join((
    f"{model_label}: {model_fps:.1f}FPS  GPU {power_w:.0f}W ",
    f"{temp_c:.0f}°/{memory_temp_c:.0f}° {used_gb:.1f}/{total_gb:.1f}G {gpu_usage_percent}%",
  ))
  return CompactEgpuStatus(True, True, text)


def chestnut_usb_speed_mbps(device_state) -> int:
  speeds = [int(device.speedMbps) for device in device_state.usbState.devices
            if (int(device.vendorId), int(device.productId)) in CHESTNUT_USB_IDS]
  return max(speeds, default=0)


def draw_egpu_status_panel(rect: rl.Rectangle, font: rl.Font, *, compact: bool) -> None:
  # Avoid constructing the global UIState when importing the pure status model
  # in tests and diagnostics.
  from openpilot.selfdrive.ui.ui_state import ui_state

  sm = ui_state.sm
  connected = resolve_egpu_connection(sm["deviceState"])
  model_seen = sm.recv_frame["modelV2"] > ui_state.started_frame
  model_alive = bool(model_seen and sm.alive["modelV2"])
  model_big = bool(model_alive and sm["modelV2"].big)
  telemetry = sm["chestnutState"]
  telemetry_valid = bool(sm.alive["chestnutState"] and sm.valid["chestnutState"] and telemetry.metricsValid)
  active_bundle = ui_state.active_bundle
  model_name = ""
  if isinstance(active_bundle, dict):
    model_name = str(active_bundle.get("internalName") or active_bundle.get("displayName") or "")
  elif active_bundle is not None:
    model_name = str(getattr(active_bundle, "internalName", "") or getattr(active_bundle, "displayName", ""))
  status = build_egpu_status(
    connected=connected, compiled=ui_state.usbgpu_compiled, loading=ui_state.usbgpu_loading,
    active=ui_state.usbgpu_active, model_alive=model_alive, model_big=model_big,
    model_name=model_name,
    loading_progress=ui_state.usbgpu_loading_progress,
    telemetry_valid=telemetry_valid, usb_speed_mbps=chestnut_usb_speed_mbps(sm["deviceState"]),
    model_fps=float(telemetry.modelFps), power_w=float(telemetry.powerDrawW),
    temp_c=float(telemetry.tempC), memory_temp_c=float(telemetry.memoryTempC),
    memory_used_mb=int(telemetry.memoryUsedMb), memory_total_mb=int(telemetry.memoryTotalMb),
    gpu_usage_percent=int(telemetry.gpuUsagePercent), gpu_clock_mhz=int(telemetry.gpuClockMhz),
    fan_speed_rpm=int(telemetry.fanSpeedRpm),
  )
  if not status.visible:
    return

  style = egpu_panel_style(compact=compact)
  lines = (status.headline, *status.details)
  panel_height = len(lines) * style.line_height
  panel = rl.Rectangle(
    rect.x + 12,
    rect.y + rect.height - style.bottom_gap - panel_height,
    style.panel_width,
    panel_height,
  )
  if style.background_alpha:
    rl.draw_rectangle_rounded(panel, 0.16, 8, rl.Color(0, 0, 0, style.background_alpha))
  for index, line in enumerate(lines):
    color = rl.Color(80, 220, 120, 245) if status.healthy and index == 0 else (
      rl.Color(255, 180, 60, 245) if index == 0 else rl.Color(255, 255, 255, 225)
    )
    rl.draw_text_ex(font, line, rl.Vector2(panel.x + 12, panel.y + index * style.line_height), style.font_size, 0, color)
