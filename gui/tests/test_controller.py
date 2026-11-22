import time
import xml.etree.ElementTree as ET


from indi_relay_gui import controller as controller_module
from indi_relay_gui.controller import CONNECT_POLL_S, Controller, Mode
from indi_relay_gui.indi import vector_from_element
from indi_relay_gui.model import IndiModel


class FakeClient:
    def __init__(self):
        self.switches = []

    def new_switch(self, device, name, members):
        self.switches.append((device, name, list(members)))

    def new_text(self, *args):
        pass

    def new_number(self, *args):
        pass


def add_light(model, device, name, member, text):
    model.apply_vector(vector_from_element(ET.fromstring(
        '<defLightVector device="%s" name="%s" state="%s">'
        '<defLight name="%s">%s</defLight></defLightVector>'
        % (device, name, text, member, text))))


def make_model(scope_safety="Ok", imu_safe=None):
    model = IndiModel()
    members = "".join(
        '<defSwitch name="PORT%02d" label="%s">Off</defSwitch>' % (index + 1, label)
        for index, label in enumerate(("Pi5", "Mount", "none")))
    model.apply_vector(vector_from_element(ET.fromstring(
        '<defSwitchVector device="KMTronic8" name="PORTS" state="IPS_OK">'
        + members + "</defSwitchVector>")))
    model.apply_vector(vector_from_element(ET.fromstring(
        '<defSwitchVector device="eulRoof" name="DOME_SHUTTER" state="IPS_OK">'
        '<defSwitch name="SHUTTER_OPEN">Off</defSwitch>'
        '<defSwitch name="SHUTTER_CLOSE">On</defSwitch>'
        "</defSwitchVector>")))
    if scope_safety is not None:
        add_light(model, "EulFocuser", "SCOPE_SAFETY", "SAFE", scope_safety)
    if imu_safe is not None:
        add_light(model, "eulRoof", "ROOF_SAFETY_STATUS", "IMU_SAFE", imu_safe)
    model.roof_state = lambda device: {"text": "closed", "state": "closed",
                                       "busy": False, "members": {}}
    return model


def make_config(starting=None, stopping=None):
    return {
        "indiserver": {"host": "127.0.0.1", "port": 7624},
        "devices": {"relay16": "Relay16", "kmtronic8": "KMTronic8", "roof": "eulRoof",
                    "focuser": "EulFocuser"},
        "relay_exclude_names": ["none"],
        "sequences": {"starting": starting or [], "stopping": stopping or []},
        "checks": {"interval_s": 15, "grace_s": 0, "items": []},
        "roof_timeout_s": 30,
        "pi5_wake": {"enabled": False},
    }


def make_controller(starting=None, stopping=None):
    model = make_model()
    client = FakeClient()
    log = []
    config = make_config(starting, stopping)
    controller = Controller(model, client, config, log.append)
    events = []
    controller.add_listener(lambda event, payload: events.append((event, payload)))
    return controller, events, client, log


def wait_until(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_starts_inactive_without_device_actions():
    controller, events, client, log = make_controller()
    assert controller.mode == Mode.INACTIVE
    assert controller.relay_actions_allowed() is False
    assert not controller.roof_move_allowed()
    assert controller.roof_stop_allowed() is False
    assert controller.set_port("kmtronic8", "Pi5", True) is False
    assert client.switches == []
    assert events[0][0] == "mode_rejected"


def test_static_mode_gates_relay_and_roof():
    controller, events, client, log = make_controller()
    assert controller.set_mode(Mode.STATIC) is True
    assert controller.mode == Mode.STATIC
    assert controller.set_port("kmtronic8", "Pi5", True) is True
    assert client.switches == [("KMTronic8", "PORTS", [("PORT01", True)])]
    controller.send_roof_motion("SHUTTER_OPEN")
    assert client.switches[-1] == ("eulRoof", "DOME_SHUTTER", [("SHUTTER_OPEN", True)])
    controller.roof_stop()
    assert client.switches[-1] == ("eulRoof", "DOME_ABORT_MOTION", [("ABORT", True)])


def test_running_mode_blocks_relay_and_roof_but_allows_stop():
    controller, events, client, log = make_controller(starting=[{"step": "wait", "seconds": 0.05}])
    controller.set_mode(Mode.STARTING)
    assert wait_until(lambda: controller.mode == Mode.RUNNING)
    assert controller.set_port("kmtronic8", "Pi5", True) is False
    assert controller.send_roof_motion("SHUTTER_OPEN") is False
    assert controller.roof_stop_allowed() is True
    assert controller.allowed_mode_changes() == [Mode.STOPPING]


def test_illegal_transition_rejected():
    controller, events, client, log = make_controller()
    assert controller.set_mode(Mode.RUNNING) is False
    assert any(event == "mode_rejected" for event, _ in events)


def test_starting_runs_sequence_and_reaches_running():
    controller, events, client, log = make_controller(starting=[{"step": "wait", "seconds": 0.05}])
    controller.set_mode(Mode.STARTING)
    assert controller.mode == Mode.STARTING
    assert wait_until(lambda: controller.mode == Mode.RUNNING)
    assert any("operating mode: running" in line for line in log)


def test_starting_failure_stays_in_starting():
    controller, events, client, log = make_controller(
        starting=[{"step": "relay_on", "device": "kmtronic8", "port": "nonexistent"}])
    controller.set_mode(Mode.STARTING)
    assert wait_until(lambda: any(e == "sequence_failed" for e, _ in events))
    assert controller.mode == Mode.STARTING
    assert controller.allowed_mode_changes() == []


def test_stopping_sequence_returns_to_inactive():
    controller, events, client, log = make_controller(
        starting=[{"step": "wait", "seconds": 0.05}],
        stopping=[{"step": "wait", "seconds": 0.05}])
    controller.set_mode(Mode.STARTING)
    assert wait_until(lambda: controller.mode == Mode.RUNNING)
    controller.set_mode(Mode.STOPPING)
    assert controller.mode == Mode.STOPPING
    assert wait_until(lambda: controller.mode == Mode.INACTIVE)


def test_empty_starting_sequence_goes_straight_to_running():
    controller, events, client, log = make_controller()
    controller.set_mode(Mode.STARTING)
    assert controller.mode == Mode.RUNNING


def test_abort_during_starting_returns_to_inactive():
    controller, events, client, log = make_controller(starting=[{"step": "wait", "seconds": 30}])
    controller.set_mode(Mode.STARTING)
    time.sleep(0.2)
    assert controller.mode == Mode.STARTING
    assert controller.abort_sequence() is True
    assert controller.mode == Mode.INACTIVE
    assert any(e == "sequence_aborted" for e, _ in events)


def test_devices_ready_lists_missing_devices():
    controller, events, client, log = make_controller()
    missing = controller.devices_ready()
    assert "Relay16" in missing
    assert "KMTronic8" not in missing
    assert "eulRoof" not in missing
    assert "EulFocuser" not in missing


def test_ensure_device_connected_sends_full_connect_vector_once():
    controller, events, client, log = make_controller()
    assert client.switches == []
    controller.ensure_device_connected("EulFocuser")
    assert client.switches == [
        ("EulFocuser", "CONNECTION", [("CONNECT", True), ("DISCONNECT", False)])]
    controller.ensure_device_connected("EulFocuser")
    assert len(client.switches) == 1
    controller.ensure_device_connected("NotAConfiguredDevice")
    assert len(client.switches) == 1
    assert any("requesting CONNECTION/CONNECT" in line
               for line in log if "EulFocuser" in line)


def connect_count(client, device):
    return sum(1 for dev, name, _ in client.switches
               if name == "CONNECTION" and dev == device)


def test_poll_device_connections_retries_until_acknowledged():
    controller, events, client, log = make_controller()
    start = 1000.0
    controller.poll_device_connections(start)
    assert connect_count(client, "Relay16") == 1
    controller.poll_device_connections(start + 1.0)
    assert connect_count(client, "Relay16") == 1
    controller.poll_device_connections(start + CONNECT_POLL_S)
    assert connect_count(client, "Relay16") == 2
    controller.note_connection_state("Relay16", True)
    controller.poll_device_connections(start + 2 * CONNECT_POLL_S)
    assert connect_count(client, "Relay16") == 2


def test_external_disconnect_rearms_connect():
    controller, events, client, log = make_controller()
    start = 2000.0
    controller.poll_device_connections(start)
    controller.note_connection_state("Relay16", True)
    assert controller.device_connection_status("Relay16") == "connected"
    controller.note_connection_state("Relay16", False)
    assert controller.device_connection_status("Relay16") == "waiting"
    controller.poll_device_connections(start + 11.0)
    assert connect_count(client, "Relay16") == 2
    assert any("requesting CONNECTION/CONNECT" in line for line in log)


def test_device_connection_status_transitions():
    controller, events, client, log = make_controller()
    assert controller.device_connection_status("Relay16") == "waiting"
    controller.poll_device_connections(1000.0)
    assert controller.device_connection_status("Relay16") == "connecting"
    controller.note_connection_state("Relay16", True)
    assert controller.device_connection_status("Relay16") == "connected"


def test_forget_device_allows_reconnect():
    controller, events, client, log = make_controller()
    controller.ensure_device_connected("EulFocuser")
    controller.forget_device("EulFocuser", "DEF_SWITCH")
    controller.ensure_device_connected("EulFocuser")
    assert len(client.switches) == 1
    controller.forget_device("EulFocuser")
    controller.ensure_device_connected("EulFocuser")
    assert len(client.switches) == 2


def test_scope_safe_from_focuser_safety_light():
    controller, events, client, log = make_controller()
    assert controller.scope_safe() is True
    add_light(controller.model, "EulFocuser", "SCOPE_SAFETY", "SAFE", "Alert")
    assert controller.scope_safe() is False


def test_scope_safe_falls_back_to_roof_imu():
    model = make_model(scope_safety=None, imu_safe="Ok")
    controller = Controller(model, FakeClient(), make_config(), lambda *_: None)
    assert controller.scope_safe() is True


def test_scope_safe_defaults_to_false_without_data():
    model = make_model(scope_safety=None, imu_safe=None)
    controller = Controller(model, FakeClient(), make_config(), lambda *_: None)
    assert controller.scope_safe() is False


def test_roof_movable_gates_on_motor_power():
    controller, events, client, log = make_controller()
    controller.set_mode(Mode.STATIC)
    assert controller.roof_power_ok() is None
    assert controller.roof_movable() is True
    assert controller.roof_block_reason() == ""

    add_light(controller.model, "eulRoof", "ROOF_SAFETY_STATUS", "MOTOR_POWER", "Busy")
    assert controller.roof_power_ok() is False
    assert controller.roof_movable() is False
    assert "roof motor power off" in controller.roof_block_reason()

    add_light(controller.model, "eulRoof", "ROOF_SAFETY_STATUS", "MOTOR_POWER", "Ok")
    assert controller.roof_movable() is True
    assert controller.roof_block_reason() == ""


def test_roof_block_reason_mentions_mode_and_safety():
    controller, events, client, log = make_controller()
    reason = controller.roof_block_reason()
    assert "not in static mode" in reason
    add_light(controller.model, "EulFocuser", "SCOPE_SAFETY", "SAFE", "Alert")
    assert "scope safety not ok" in controller.roof_block_reason()


def test_roof_movable_false_when_roof_offline():
    controller, events, client, log = make_controller()
    controller.set_mode(Mode.STATIC)
    del controller.model.properties[("eulRoof", "DOME_SHUTTER")]
    assert controller.roof_movable() is False
    assert "roof driver offline" in controller.roof_block_reason()


def make_wake_controller(enabled=True):
    config = make_config()
    config["pi5_wake"] = {"enabled": enabled, "host": "192.0.2.10", "port": 80,
                          "user": "", "password": "", "relay": 3,
                          "ping_host": "192.0.2.9", "timeout_s": 1, "wait_s": 60}
    logs = []
    controller = Controller(make_model(), FakeClient(), config, logs.append)
    events = []
    controller.add_listener(lambda event, payload: events.append((event, payload)))
    return controller, events, logs


def test_wake_pi5_switches_relay_then_reconnects(monkeypatch):
    controller, events, logs = make_wake_controller()
    pings = iter([False, True])
    monkeypatch.setattr(controller_module.kmtronic, "host_reachable",
                        lambda host, timeout_s: next(pings))
    switches = []

    def fake_power_on(*args):
        switches.append(args)
        return "/FF0301", False

    monkeypatch.setattr(controller_module.kmtronic, "power_on", fake_power_on)
    monkeypatch.setattr(controller_module, "PI5_POLL_S", 0.01)

    assert controller.set_mode(Mode.STATIC) is True
    assert wait_until(lambda: "reconnect_requested" in [e for e, _ in events])
    assert switches == [("192.0.2.10", 80, "", "", 3, 1)]
    assert controller.client.switches == []
    assert "switching relay 3 on 192.0.2.10 directly" in " ".join(logs)
    assert "no relay action needed" not in " ".join(logs)


def test_wake_pi5_cycles_powered_relay(monkeypatch):
    controller, events, logs = make_wake_controller()
    monkeypatch.setattr(controller_module.kmtronic, "host_reachable", lambda *a: False)
    monkeypatch.setattr(controller_module.kmtronic, "power_on",
                        lambda *a: ("/FF0300, then /FF0301", True))
    monkeypatch.setattr(controller_module.kmtronic, "set_relay", lambda *a: ("", False))

    assert controller.set_mode(Mode.STATIC) is True
    assert wait_until(lambda: any("cycled" in line for line in logs))
    assert not any("could not switch" in line for line in logs)


def test_wake_pi5_skipped_when_disabled(monkeypatch):
    controller, events, logs = make_wake_controller(enabled=False)
    called = []
    monkeypatch.setattr(controller_module.kmtronic, "host_reachable",
                        lambda *a: called.append("ping"))
    monkeypatch.setattr(controller_module.kmtronic, "power_on",
                        lambda *a: called.append("relay"))

    assert controller.set_mode(Mode.STATIC) is True
    time.sleep(0.2)
    assert called == []
    assert events == [("mode_changed", Mode.STATIC)]


def test_wake_pi5_skipped_when_pi5_pings(monkeypatch):
    controller, events, logs = make_wake_controller()
    monkeypatch.setattr(controller_module.kmtronic, "host_reachable", lambda *a: True)
    called = []
    monkeypatch.setattr(controller_module.kmtronic, "power_on",
                        lambda *a: called.append("relay"))

    assert controller.set_mode(Mode.STATIC) is True
    time.sleep(0.2)
    assert called == []
    assert "no relay action needed" in " ".join(logs)
    assert "reconnect_requested" not in [e for e, _ in events]
