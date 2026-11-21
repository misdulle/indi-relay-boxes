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
    assert kinds["imu_udp"] == "udp"
    assert config["checks"]["items"][1]["recovery"][0]["step"] == "relay_cycle"


def test_defaults_fill_missing_sections():
    config = config_module.normalize({})
    assert config["indiserver"]["port"] == config_module.DEFAULT_PORT
    assert config["checks"]["interval_s"] == 15
    assert config["sequences"]["starting"] == []
    assert config["roof_timeout_s"] == 180


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
