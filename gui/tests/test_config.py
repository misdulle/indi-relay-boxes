import pytest

from indi_relay_gui import config as config_module


EXAMPLE = (config_module.__file__.rsplit("/", 1)[0],)


def _example_text():
    from pathlib import Path
    path = Path(config_module.__file__).resolve().parent / "config.example.yaml"
    return path.read_text(), path


def test_load_example_config():
    text, _ = _example_text()
    config = config_module.normalize(__import__("yaml").safe_load(text))
    assert config["indiserver"] == {"host": "192.168.1.29", "port": 7624}
    assert config["devices"]["roof"] == "eulRoof"
    assert config["relay_exclude_names"] == ["none"]
    assert [s["step"] for s in config["sequences"]["starting"]][:2] == ["relay_on", "wait"]
    kinds = {c["name"]: c["kind"] for c in config["checks"]["items"]}
    assert kinds["pi5_ping"] == "ping"
    assert kinds["imu_udp"] == "quaternion"
    assert config["checks"]["items"][1]["recovery"][0]["step"] == "relay_cycle"
    assert config["pi5_wake"]["enabled"] is True
    assert config["pi5_wake"]["relay"] == 1
    assert config["pi5_wake"]["ping_host"] == "192.168.1.29"
    assert config["mount3d_view"] == {"az": 200.0, "alt": 25.0}
    cameras = config["shed_cameras"]["cameras"]
    assert cameras["cam1"]["url"] == (
        "rtsp://user:pass@192.168.1.20:554/Streaming/Channels/2")
    assert cameras["cam1"]["enabled"] is True
    assert cameras["cam1"]["fps"] == 8.0
    assert cameras["cam2"] == {"enabled": False, "url": "", "fps": 8.0}


def test_defaults_fill_missing_sections():
    config = config_module.normalize({})
    assert config["indiserver"]["port"] == config_module.DEFAULT_PORT
    assert config["checks"]["interval_s"] == 15
    assert config["sequences"]["starting"] == []
    assert config["roof_timeout_s"] == 180
    assert config["scope_view_rotation"] == 200.0


def test_scope_view_rotation_normalized():
    config = config_module.normalize({"scope_view_rotation": "175"})
    assert config["scope_view_rotation"] == 175.0


def test_shed_cameras_defaults_empty():
    config = config_module.normalize({})
    assert config["shed_cameras"] == {"fps": 8.0, "cameras": {}}


def test_shed_cameras_normalized_sorted_with_own_fps():
    config = config_module.normalize({
        "shed_cameras": {"fps": 5, "cameras": {
            "cam2": {"enabled": True, "url": "rtsp://host/2"},
            "cam1": {"url": "rtsp://host/1", "fps": 22}}}})
    assert list(config["shed_cameras"]["cameras"]) == ["cam1", "cam2"]
    assert config["shed_cameras"]["cameras"]["cam1"] == {
        "enabled": False, "url": "rtsp://host/1", "fps": 22.0}
    assert config["shed_cameras"]["fps"] == 5.0
    assert config["shed_cameras"]["cameras"]["cam2"]["fps"] == 5.0


def test_shed_camera_fps_clamped():
    config = config_module.normalize({"shed_cameras": {"cameras": {"a": {"fps": 99}}}})
    assert config["shed_cameras"]["cameras"]["a"]["fps"] == 30.0
    config = config_module.normalize({"shed_cameras": {"cameras": {"a": {"fps": 0}}}})
    assert config["shed_cameras"]["cameras"]["a"]["fps"] == 1.0
    config = config_module.normalize({"shed_cameras": {"fps": 99}})
    assert config["shed_cameras"]["fps"] == 30.0


def test_shed_camera_legacy_block_becomes_cam1():
    config = config_module.normalize({
        "shed_camera": {"enabled": "yes", "url": "rtsp://host/cam", "fps": "4"}})
    assert config["shed_cameras"]["cameras"]["cam1"] == {
        "enabled": True, "url": "rtsp://host/cam", "fps": 4.0}
    assert "shed_camera" not in config


def test_shed_cameras_bad_types_rejected():
    with pytest.raises(ValueError, match="shed_cameras"):
        config_module.normalize({"shed_cameras": "on"})
    with pytest.raises(ValueError, match="cameras"):
        config_module.normalize({"shed_cameras": {"cameras": ["a"]}})
    with pytest.raises(ValueError, match="shed_cameras.a"):
        config_module.normalize({"shed_cameras": {"cameras": {"a": 3}}})


def test_mount3d_view_defaults():
    config = config_module.normalize({})
    assert config["mount3d_view"] == {"az": 200.0, "alt": 25.0}


def test_mount3d_view_normalized():
    config = config_module.normalize({"mount3d_view": {"az": 560, "alt": -90}})
    assert config["mount3d_view"]["az"] == 200.0
    assert config["mount3d_view"]["alt"] == -80.0


def test_mount3d_view_bad_type_rejected():
    with pytest.raises(ValueError, match="mount3d_view"):
        config_module.normalize({"mount3d_view": [200, 25]})


def test_relay_exclude_names_are_normalized():
    config = config_module.normalize({"relay_exclude_names": ["None", "SPARE"]})
    assert config["relay_exclude_names"] == ["none", "spare"]


def test_unknown_check_type_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"checks": {"weird": {"type": "snmp"}}})


def test_missing_check_parameters_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"checks": {"p": {"type": "ping"}}})
    with pytest.raises(ValueError):
        config_module.normalize({"checks": {"u": {"type": "udp"}}})


def test_step_without_step_key_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"sequences": {"starting": [{"seconds": 2}]}})


def test_imu_calibration_keys_normalized():
    config = config_module.normalize({
        "imu_park_quaternion": [0.108337, -0.022705, -0.081787, 0.990479]})
    assert config["imu_park_quaternion"] == (0.108337, -0.022705, -0.081787, 0.990479)
    assert config["imu_optical_axis"] == "+x"
    assert config["devices"]["mount"] == "EQMod Mount"


def test_bad_imu_calibration_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"imu_park_quaternion": [1.0, 2.0]})
    with pytest.raises(ValueError):
        config_module.normalize({"imu_optical_axis": "+w"})


def test_quaternion_check_parameters_validated():
    with pytest.raises(ValueError):
        config_module.normalize({"checks": {"q": {"type": "quaternion"}}})
    config = config_module.normalize({
        "checks": {"q": {"type": "quaternion", "port": 2346, "timeout_s": 15}}})
    item = config["checks"]["items"][0]
    assert item["kind"] == "quaternion"
    assert item["port"] == 2346
    assert item["bind"] == "0.0.0.0"


def test_pi5_wake_defaults_off():
    config = config_module.normalize({})
    assert config["pi5_wake"] == {
        "enabled": False, "host": "192.168.1.23", "port": 80, "user": "admin",
        "password": "", "relay": 1, "ping_host": None, "timeout_s": 3.0,
        "wait_s": 90.0}


def test_pi5_wake_normalized():
    config = config_module.normalize({
        "pi5_wake": {"enabled": "yes", "host": "192.168.1.40", "relay": "4",
                     "ping_host": "192.168.1.30", "timeout_s": "2", "wait_s": 0}})
    wake = config["pi5_wake"]
    assert wake["enabled"] is True
    assert wake["host"] == "192.168.1.40"
    assert wake["relay"] == 4
    assert wake["ping_host"] == "192.168.1.30"
    assert wake["timeout_s"] == 2.0
    assert wake["wait_s"] == 0.0


def test_pi5_wake_relay_range_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"pi5_wake": {"relay": 0}})
    with pytest.raises(ValueError):
        config_module.normalize({"pi5_wake": {"relay": 9}})


def test_pi5_wake_port_range_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"pi5_wake": {"port": 0}})
    with pytest.raises(ValueError):
        config_module.normalize({"pi5_wake": {"port": 70000}})


def test_pi5_wake_bad_type_rejected():
    with pytest.raises(ValueError):
        config_module.normalize({"pi5_wake": "on"})
