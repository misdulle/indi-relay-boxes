"""YAML configuration handling with defaults and validation."""

import copy
import os
import re

import yaml

DEFAULT_HOST = "192.168.1.29"
DEFAULT_PORT = 7624

DEFAULTS = {
    "indiserver": {"host": DEFAULT_HOST, "port": DEFAULT_PORT},
    "devices": {"relay16": "Relay16", "kmtronic8": "KMTronic8", "roof": "eulRoof",
                "focuser": "EulFocuser", "mount": "EQMod Mount"},
    "relay_exclude_names": ["none"],
    "sequences": {"starting": [], "stopping": []},
    "checks": {"interval_s": 15, "grace_s": 120},
    "roof_timeout_s": 180,
    "imu_park_quaternion": None,       # (w, x, y, z) captured parked; enables the IMU drawing
    "imu_optical_axis": "+x",          # IMU body axis pointing along the scope tube
    "scope_view_rotation": 200.0,      # top-view azimuths rotated by this many degrees
                                       # clockwise (200 = south up + 20 for the shed)
    "shed_cameras": {"fps": 8.0, "cameras": {}},
    "mount3d_view": {"az": 200.0, "alt": 25.0},
    "pi5_wake": {"enabled": False, "host": "192.168.1.23", "port": 80,
                 "user": "admin", "password": "", "relay": 1,
                 "timeout_s": 3, "wait_s": 90},
}

_OPTICAL_AXES = ("+x", "-x", "+y", "-y", "+z", "-z")


DEFAULT_CONFIG_PATH = os.path.join(
    os.path.expanduser("~"), ".config", "indi-relay-boxes", "config.yaml")


def deep_merge(base, extra):
    result = copy.deepcopy(base)
    for key, value in (extra or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def normalize(raw):
    config = deep_merge(DEFAULTS, raw)
    config["indiserver"] = _section(config["indiserver"], ("host", "port"))
    config["indiserver"]["host"] = str(config["indiserver"]["host"])
    config["indiserver"]["port"] = int(config["indiserver"]["port"])
    if not config["devices"] or not isinstance(config["devices"], dict):
        raise ValueError("devices: must be a map of alias -> INDI device names")
    config["devices"] = dict(config["devices"])
    for required in ("roof",):
        if required not in config["devices"]:
            raise ValueError("devices: missing '%s'" % required)
    if config["relay_exclude_names"] is None:
        config["relay_exclude_names"] = []
    config["relay_exclude_names"] = [str(n).lower() for n in config["relay_exclude_names"]]
    config["sequences"] = config["sequences"] or {}
    for key in ("starting", "stopping"):
        config["sequences"][key] = _steps(config["sequences"].get(key))
    park_quat = config.get("imu_park_quaternion")
    if park_quat is not None:
        if (not isinstance(park_quat, (list, tuple)) or len(park_quat) != 4):
            raise ValueError("imu_park_quaternion: expected [w, x, y, z]")
        config["imu_park_quaternion"] = tuple(float(v) for v in park_quat)
    axis = config.get("imu_optical_axis", "+x") or "+x"
    if axis not in _OPTICAL_AXES:
        raise ValueError("imu_optical_axis: use one of %s" % ", ".join(_OPTICAL_AXES))
    config["imu_optical_axis"] = axis
    config["scope_view_rotation"] = float(
        config.get("scope_view_rotation", DEFAULTS["scope_view_rotation"]))
    legacy = config.pop("shed_camera", None)
    cams = config.get("shed_cameras") or {}
    if not isinstance(cams, dict):
        raise ValueError("shed_cameras: expected a mapping")
    default_fps = min(30.0, max(1.0, float(cams.get("fps", 8.0))))
    cameras = cams.get("cameras") or {}
    if not isinstance(cameras, dict):
        raise ValueError("shed_cameras.cameras: expected a mapping")
    cameras = dict(cameras)
    if not cameras and isinstance(legacy, dict) and legacy.get("url"):
        cameras["cam1"] = legacy   # old single-camera config
    normalized_cams = {}
    for name, settings in sorted(cameras.items()):
        if settings is None:
            settings = {}
        if not isinstance(settings, dict):
            raise ValueError("shed_cameras.%s: expected a mapping" % name)
        normalized_cams[str(name)] = {
            "enabled": bool(settings.get("enabled", False)),
            "url": str(settings.get("url", "")),
            "fps": min(30.0, max(1.0, float(settings.get("fps", default_fps)))),
        }
    config["shed_cameras"] = {"fps": default_fps, "cameras": normalized_cams}
    view3d = config.get("mount3d_view") or {}
    if not isinstance(view3d, dict):
        raise ValueError("mount3d_view: expected a mapping")
    config["mount3d_view"] = {
        "az": float(view3d.get("az", 200.0)) % 360.0,
        "alt": max(-80.0, min(80.0, float(view3d.get("alt", 25.0)))),
    }
    wake = config.get("pi5_wake") or {}
    if not isinstance(wake, dict):
        raise ValueError("pi5_wake: expected a mapping")
    wake.setdefault("ping_host", None)
    wake["enabled"] = bool(wake.get("enabled", False))
    wake["host"] = str(wake.get("host", DEFAULTS["pi5_wake"]["host"]))
    wake["port"] = int(wake.get("port", 80))
    if not 1 <= wake["port"] <= 65535:
        raise ValueError("pi5_wake.port: out of range")
    wake["user"] = str(wake.get("user", DEFAULTS["pi5_wake"]["user"]))
    wake["password"] = str(wake.get("password", ""))
    wake["relay"] = int(wake.get("relay", 1))
    if not 1 <= wake["relay"] <= 8:
        raise ValueError("pi5_wake.relay: KMTronic8 has relays 1..8")
    wake["timeout_s"] = max(1.0, float(wake.get("timeout_s", 3)))
    wake["wait_s"] = max(0.0, float(wake.get("wait_s", 90)))
    config["pi5_wake"] = wake
    checks = config["checks"] or {}
    config["checks"] = {
        "interval_s": max(1, int(checks.get("interval_s", DEFAULTS["checks"]["interval_s"]))),
        "grace_s": max(0, float(checks.get("grace_s", DEFAULTS["checks"]["grace_s"]))),
        "items": [],
    }
    for name, spec in checks.items():
        if name in ("interval_s", "grace_s") or not isinstance(spec, dict):
            continue
        config["checks"]["items"].append(normalize_check(name, spec))
    config["roof_timeout_s"] = max(1, int(config["roof_timeout_s"]))
    return config


def normalize_check(name, spec):
    check = dict(spec)
    check["name"] = str(name)
    check["type"] = str(check.get("type") or name)
    known = {"pi5_ping": "ping", "imu_udp": "udp"}
    check["kind"] = check["type"] if check["type"] in ("ping", "udp") else known.get(check["type"], check["type"])
    check["enabled"] = bool(check.get("enabled", True))
    check["failures"] = max(1, int(check.get("failures", 3)))
    check["timeout_s"] = max(1, int(check.get("timeout_s", 5)))
    check["recovery"] = _steps(check.get("recovery"))
    if check["kind"] == "ping":
        if not check.get("host"):
            raise ValueError("checks.%s: ping check needs a host" % name)
    elif check["kind"] == "udp":
        if not check.get("port"):
            raise ValueError("checks.%s: udp check needs a port" % name)
        check["port"] = int(check["port"])
        check["bind"] = str(check.get("bind", "0.0.0.0"))
    elif check["kind"] == "quaternion":
        if not check.get("port"):
            raise ValueError("checks.%s: quaternion check needs a port" % name)
        check["port"] = int(check["port"])
        check["bind"] = str(check.get("bind", "0.0.0.0"))
    else:
        raise ValueError("checks.%s: unknown check type %r (use ping or udp)" % (name, check["type"]))
    return check


def _section(value, keys):
    if not isinstance(value, dict):
        raise ValueError("expected a mapping with keys %s" % ", ".join(keys))
    return {key: value.get(key) for key in keys}


def _steps(value):
    if value is None:
        return []
    if not isinstance(value, list):
        raise ValueError("sequence steps: expected a list of {step: ...} mappings")
    steps = []
    for index, item in enumerate(value):
        if not isinstance(item, dict) or "step" not in item:
            raise ValueError("sequence step %d: expected a mapping with a 'step' key" % (index + 1))
        steps.append(dict(item))
    return steps


def load(path):
    with open(path, "r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        raise ValueError("config root must be a mapping")
    return normalize(raw)


def _format_scalar(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float) and value.is_integer():
        return "%.1f" % value
    return repr(value)


def update_section_values(text, section, updates):
    """Edit `key: value` pairs of a top-level mapping in yaml *text*.

    Comments and everything outside the section are preserved; missing keys
    are appended to the section, a missing section is appended at the end.
    """
    pattern = re.compile(r"(\s*)(%s)\s*:.*?(\r?\n|$)" % "|".join(
        re.escape(key) for key in updates))
    lines = text.splitlines(keepends=True)
    start = None
    for index, line in enumerate(lines):
        if line.strip().startswith("#"):
            continue
        if (line.split(":", 1)[0].strip() == section
                and not line.startswith((" ", "\t"))):
            start = index
            break
    result = list(lines)
    if start is None:
        block = "%s:\n" % section
        if text and not text.endswith(("\n", "\r")):
            block = "\n" + block
        for key, value in updates.items():
            block += "  %s: %s\n" % (key, _format_scalar(value))
        return text + block
    end = start + 1
    while end < len(lines) and (lines[end].startswith((" ", "\t"))
                                or not lines[end].strip()
                                or lines[end].strip().startswith("#")):
        end += 1
    replaced = set()
    for index in range(start + 1, end):
        match = pattern.match(lines[index])
        if match and match.group(2) not in replaced:
            key = match.group(2)
            tail = match.group(3) if match.group(3) else ""
            result[index] = "%s%s: %s%s" % (
                match.group(1), key, _format_scalar(updates[key]), tail)
            replaced.add(key)
    insert_at = end
    for key, value in updates.items():
        if key not in replaced:
            result.insert(insert_at, "  %s: %s\n" % (key, _format_scalar(value)))
            insert_at += 1
    return "".join(result)


def ensure_config(path, template_text):
    if os.path.exists(path):
        return False
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(template_text)
    return True
