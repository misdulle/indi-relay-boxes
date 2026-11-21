import time
import xml.etree.ElementTree as ET


from indi_relay_gui.controller import Controller, Mode
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


def make_model():
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
    model.roof_state = lambda device: {"text": "closed", "state": "closed",
                                       "busy": False, "members": {}}
    return model


def make_config(starting=None, stopping=None):
    return {
        "indiserver": {"host": "127.0.0.1", "port": 7624},
        "devices": {"relay16": "Relay16", "kmtronic8": "KMTronic8", "roof": "eulRoof"},
        "relay_exclude_names": ["none"],
        "sequences": {"starting": starting or [], "stopping": stopping or []},
        "checks": {"interval_s": 15, "grace_s": 0, "items": []},
        "roof_timeout_s": 30,
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
