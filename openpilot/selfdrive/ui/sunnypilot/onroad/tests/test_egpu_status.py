from openpilot.selfdrive.ui.egpu_status import (
  build_egpu_sidebar_status,
  build_egpu_status,
  build_compact_egpu_status,
  build_compact_jetlink_status,
  build_jetlink_gpu_status,
  classify_egpu_link_state,
  egpu_icon_visible,
  egpu_panel_style,
  jetlink_gpu_icon_severity,
  resolve_egpu_connection,
)


class FakeDeviceState:
  def __init__(self, present: bool):
    self.chestnutPresent = present


def test_physical_disconnect_immediately_clears_egpu_connection():
  device_state = FakeDeviceState(True)
  assert resolve_egpu_connection(device_state)

  device_state.chestnutPresent = False
  assert not resolve_egpu_connection(device_state)


def test_connected_egpu_icon_stays_visible_onroad():
  assert egpu_icon_visible(connected=True)
  assert not egpu_icon_visible(connected=False)


def test_jetlink_gpu_tile_follows_the_phone_accelerator():
  offline = build_jetlink_gpu_status(state="disconnected", present=False)
  assert (offline.value, offline.severity) == ("OFFLINE", "disabled")

  waiting = build_jetlink_gpu_status(state="loading", present=True)
  assert (waiting.value, waiting.severity) == ("WAIT", "progress")

  building = build_jetlink_gpu_status(state="loading", present=True,
                                      progress={"stage": "build", "frac": 0.42})
  assert (building.value, building.severity) == ("LOAD 42%", "progress")

  ready = build_jetlink_gpu_status(state="ready", present=True)
  assert (ready.value, ready.severity) == ("READY", "good")

  active = build_jetlink_gpu_status(state="active", present=True, frame_ms=21.4, model="Cinque Terre")
  assert (active.value, active.severity) == ("ACTIVE", "good")
  assert "21 ms/帧" in active.detail
  assert "Cinque Terre" in active.detail

  failed = build_jetlink_gpu_status(state="failed", present=True)
  assert (failed.value, failed.severity) == ("ERR", "danger")


def test_jetlink_gpu_icon_is_green_only_while_the_phone_drives():
  assert jetlink_gpu_icon_severity(state="active", present=True) == "good"
  assert jetlink_gpu_icon_severity(state="ready", present=True) == "warning"
  assert jetlink_gpu_icon_severity(state="failed", present=True) == "danger"
  assert jetlink_gpu_icon_severity(state="active", present=False) == "disabled"


def test_jetlink_bottom_line_mirrors_the_gpu_line():
  assert not build_compact_jetlink_status(state="disconnected", present=False).visible

  loading = build_compact_jetlink_status(state="loading", present=True,
                                         progress={"stage": "build", "frac": 0.37})
  assert (loading.text, loading.severity) == ("JetLink: LOAD 37%", "progress")

  uploading = build_compact_jetlink_status(state="loading", present=True,
                                           progress={"stage": "upload", "frac": 0.5})
  assert uploading.text == "JetLink: UPLOAD 50%"

  joining = build_compact_jetlink_status(state="loading", present=True)
  assert joining.text == "JetLink: JOIN"

  ready = build_compact_jetlink_status(state="ready", present=True)
  assert (ready.text, ready.severity) == ("JetLink: READY", "good")

  active = build_compact_jetlink_status(
    state="active", present=True, model_fps=20.0, frame_ms=24.6,
    device="SM8750", backend="ORT", usb_speed="USB5",
  )
  assert active.text == "JetLink: 20.0FPS 25ms USB5 SM8750 ORT"
  assert active.severity == "good"

  failed = build_compact_jetlink_status(state="failed", present=True)
  assert (failed.text, failed.severity) == ("JetLink: ERR", "danger")


def test_jetlink_hardware_record_is_shortened_and_cached():
  import json
  import tempfile
  from pathlib import Path
  import openpilot.selfdrive.ui.egpu_status as egpu_status

  with tempfile.TemporaryDirectory() as tmp:
    server = Path(tmp) / "server.json"
    record = Path(tmp) / "status.json"
    server.write_text(json.dumps({"device": "htp-whole-SM8750", "backend": "ort"}))
    record.write_text(json.dumps({"speed": "super-speed"}))

    saved = (egpu_status.JETLINK_SERVER_RECORD, egpu_status.JETLINK_STATUS_RECORD,
             egpu_status._hardware_cache, egpu_status._hardware_loaded)
    try:
      egpu_status.JETLINK_SERVER_RECORD = server
      egpu_status.JETLINK_STATUS_RECORD = record
      egpu_status._hardware_cache = (0.0, egpu_status.JetlinkHardware())
      egpu_status._hardware_loaded = False

      hardware = egpu_status.read_jetlink_hardware(now=100.0)
      assert (hardware.device, hardware.backend, hardware.usb_speed) == ("SM8750", "ORT", "USB5")

      server.write_text(json.dumps({"device": "htp-whole-SM8650", "backend": "ort"}))
      assert egpu_status.read_jetlink_hardware(now=100.5).device == "SM8750"
      assert egpu_status.read_jetlink_hardware(now=102.0).device == "SM8650"
    finally:
      (egpu_status.JETLINK_SERVER_RECORD, egpu_status.JETLINK_STATUS_RECORD,
       egpu_status._hardware_cache, egpu_status._hardware_loaded) = saved


def test_egpu_panel_is_at_least_twice_as_large_with_transparent_background():
  style = egpu_panel_style(compact=True)
  assert style.font_size >= 48
  assert style.line_height >= 58
  assert style.background_alpha == 0


def test_egpu_status_explains_fallback_reason():
  status = build_egpu_status(
    connected=True, compiled=True, loading=False, active=False,
    model_alive=True, model_big=False, telemetry_valid=False, usb_speed_mbps=5000,
    model_name="IDM",
  )
  assert status.visible
  assert not status.healthy
  assert status.headline.startswith("IDM · ")
  assert "回退小模型" in status.headline
  assert "USB" not in " ".join((status.headline, *status.details))


def test_egpu_status_reports_live_model_and_gpu_metrics():
  status = build_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=True, telemetry_valid=True,
    model_name="IDM",
    usb_speed_mbps=5000, model_fps=19.8, power_w=72.0,
    temp_c=61.0, memory_temp_c=70.0, memory_used_mb=6144,
    memory_total_mb=8192, gpu_usage_percent=88, gpu_clock_mhz=2200,
    fan_speed_rpm=1450,
  )
  assert status.healthy
  assert status.headline.startswith("IDM · ")
  assert "19.8 FPS" in status.headline
  assert status.details[0] == "功耗 72 W"
  assert "6.0/8.0 GB" in status.details[1]
  assert "88%" in status.details[2]
  assert "2200 MHz" in status.details[2]
  assert "AMD" not in " ".join((status.headline, *status.details))
  assert "USB 5000" not in " ".join((status.headline, *status.details))


def test_compact_bottom_status_contains_only_requested_gpu_metrics():
  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=False, active=True,
    model_alive=True, model_big=True, telemetry_valid=True, model_name="IDM",
    model_fps=19.8, power_w=72.0, temp_c=61.0, memory_temp_c=70.0,
    memory_used_mb=6144, memory_total_mb=8192, gpu_usage_percent=88,
  )

  assert status.visible
  assert status.healthy
  assert status.text == "IDM: 19.8FPS  GPU 72W 61°/70° 6.0/8.0G 88%"
  assert "USB" not in status.text
  assert "AMD" not in status.text


def test_compact_bottom_status_replaces_metrics_with_loading_progress():
  status = build_compact_egpu_status(
    connected=True, compiled=True, loading=True, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
    model_name="IDM", loading_progress=57,
  )

  assert status.text == "IDM: LOAD 57%"


def test_absent_egpu_has_no_panel():
  status = build_egpu_status(
    connected=False, compiled=False, loading=False, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
  )
  assert not status.visible


def test_loading_status_exposes_real_progress_percentage():
  panel = build_egpu_status(
    connected=True, compiled=True, loading=True, active=None,
    model_alive=False, model_big=False, telemetry_valid=False, loading_progress=64,
    model_name="IDM",
  )
  sidebar = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="ready", usb_speed_mbps=5000,
    pcie_ltssm=0x78, eject_status=None, loading=True, active=None, loading_progress=64,
  )

  assert panel.headline.startswith("IDM · ")
  assert "64%" in panel.headline
  assert sidebar.value == "LOAD 64%"
  assert "64%" in sidebar.detail


def test_optional_egpu_absence_stays_quiet_even_when_big_model_is_compiled():
  status = build_egpu_status(
    connected=False, compiled=True, loading=False, active=None,
    model_alive=False, model_big=False, telemetry_valid=False,
  )
  assert not status.visible
  assert not status.headline


def test_sidebar_makes_usb_degradation_visible_at_a_glance():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="usb_degraded", usb_speed_mbps=480,
    pcie_ltssm=None, eject_status=None, loading=False, active=None,
  )

  assert status.value == "USB 480"
  assert status.severity == "danger"
  assert "低于 5000" in status.detail


def test_sidebar_makes_pcie_failure_distinct_from_usb_failure():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="pcie_down", usb_speed_mbps=5000,
    pcie_ltssm=0, eject_status=None, loading=False, active=None,
  )

  assert status.value == "PCIE ERR"
  assert status.severity == "danger"
  assert "LTSSM 0x00" in status.detail


def test_sidebar_reports_ready_link_before_model_starts():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="ready", usb_speed_mbps=5000,
    pcie_ltssm=0x78, eject_status=None, loading=False, active=None,
  )

  assert status.value == "READY"
  assert status.severity == "good"


def test_sidebar_uses_full_model_failure_instead_of_stale_active_flag():
  status = build_egpu_sidebar_status(
    present=True, compiled=True, link_state="ready", usb_speed_mbps=5000,
    pcie_ltssm=0x78, eject_status=None, loading=False, active=True,
    model_failed=True,
  )

  assert status.value == "MODEL ERR"
  assert status.severity == "danger"


def test_sidebar_link_classification_uses_existing_chestnut_telemetry():
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=480, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0x78,
  ) == "usb_degraded"
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0,
  ) == "pcie_down"
  assert classify_egpu_link_state(
    present=True, usb_speed_mbps=5000, telemetry_alive=True, telemetry_valid=True, pcie_ltssm=0x78,
  ) == "ready"
