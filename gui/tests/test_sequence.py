import time

import pytest
import xml.etree.ElementTree as ET

from indi_relay_gui.indi import vector_from_element
from indi_relay_gui.model import IndiModel
from indi_relay_gui.sequence import StepContext, StepError, execute

DEVICES = {"kmtronic8": "KMTronic8", "roof": "eulRoof"}

SHUTTER_STATES = {
    "open": ("On", "Off"), "closed": ("Off", "On"),
    "moving": ("Off", "Off"), "unknown": ("Off", "Off"),
    "error": ("Off", "Off"),
}


def make_model(roof_state="closed"):
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
    set_roof(model, roof_state)
    return model


def set_roof(model, roof_state):
    model.state_bucket = {"value": roof_state}
    roof = SHUTTER_STATES[roof_state]
    busy = roof_state == "moving"

    def roof_state_fn(device):
        return {"text": roof_state, "state": roof_state, "busy": busy, "shutter": roof,
                "members": {}}

    model.roof_state = roof_state_fn


class FakeClient:
    def __init__(self):
        self.switches = []

    def new_switch(self, device, name, members):
        self.switches.append((device, name, list(members)))

    def new_text(self, *args):
        pass

    def new_number(self, *args):
        pass


def make_ctx(model, client, roof_state=None, clock=time.time, messages=None):
    messages = messages if messages is not None else []
    ctx = StepContext(model, client, DEVICES, messages.append,
                      roof_timeout_s=30, clock=clock)
    if roof_state is not None:
        set_roof(model, roof_state)
    return ctx


def test_relay_on_off_by_name_and_index():
    model = make_model()
    client = FakeClient()
    ctx = make_ctx(model, client)
    execute([{"step": "relay_on", "device": "kmtronic8", "port": "Pi5"},
             {"step": "relay_off", "device": "kmtronic8", "port": 2}], ctx)
    assert client.switches == [
        ("KMTronic8", "PORTS", [("PORT01", True)]),
        ("KMTronic8", "PORTS", [("PORT02", False)]),
    ]


def test_relay_cycle():
    model = make_model()
    client = FakeClient()
    waited = []
    ctx = make_ctx(model, client)
    ctx._sleeper = lambda seconds: waited.append(seconds)
    execute([{"step": "relay_cycle", "device": "kmtronic8", "port": "Pi5", "seconds": 10}], ctx)
    assert client.switches[0][2] == [("PORT01", False)]
    assert client.switches[1][2] == [("PORT01", True)]
    assert 10 in waited


def test_unknown_port_raises():
    model = make_model()
    ctx = make_ctx(model, FakeClient())
    with pytest.raises(StepError):
        execute([{"step": "relay_on", "device": "kmtronic8", "port": "nonexistent"}], ctx)


def test_unknown_device_alias_raises():
    model = make_model()
    ctx = make_ctx(model, FakeClient())
    with pytest.raises(StepError):
        execute([{"step": "relay_on", "device": "wrongbox", "port": "Pi5"}], ctx)


def test_roof_open_waits_for_state():
    model = make_model(roof_state="closed")
    client = FakeClient()
    ctx = make_ctx(model, client)
    state = {"n": 0}

    def flip(device):
        state["n"] += 1
        if state["n"] > 3:
            set_roof(model, "open")
            return {"text": "open", "state": "open", "busy": False}
        return {"text": "moving", "state": "moving", "busy": True}

    ctx.roof_state = flip
    execute([{"step": "roof_open", "timeout": 30}], ctx)
    assert client.switches == [("eulRoof", "DOME_SHUTTER", [("SHUTTER_OPEN", True)])]


def test_roof_open_timeout_raises():
    model = make_model(roof_state="closed")
    ctx = make_ctx(model, FakeClient())
    ctx.roof_state = lambda device: {"text": "moving", "state": "moving", "busy": True}
    with pytest.raises(StepError, match="did not reach open"):
        execute([{"step": "roof_open", "timeout": 0.2}], ctx)


def test_roof_open_already_open_is_noop():
    model = make_model(roof_state="open")
    client = FakeClient()
    ctx = make_ctx(model, client)
    execute([{"step": "roof_open"}], ctx)
    assert client.switches == []


def test_roof_error_state_fails_step():
    model = make_model(roof_state="closed")
    ctx = make_ctx(model, FakeClient())
    ctx.roof_state = lambda device: {"text": "error", "state": "error", "busy": False}
    with pytest.raises(StepError, match="error"):
        execute([{"step": "roof_open"}], ctx)


def test_roof_stop_sends_abort():
    model = make_model()
    client = FakeClient()
    ctx = make_ctx(model, client)
    execute([{"step": "roof_stop"}], ctx)
    assert client.switches == [("eulRoof", "DOME_ABORT_MOTION", [("ABORT", True)])]


def test_wait_sleeps():
    model = make_model()
    waited = []
    ctx = make_ctx(model, FakeClient())
    ctx._sleeper = lambda seconds: waited.append(seconds)
    execute([{"step": "wait", "seconds": 2}], ctx)
    assert waited == [2.0]


def test_abort_stops_sequence_immediately():
    model = make_model()
    ctx = make_ctx(model, FakeClient())

    def blow_up(seconds):
        ctx.abort()
        raise StepError("aborted")

    ctx.sleep = blow_up
    with pytest.raises(StepError, match="aborted"):
        execute([{"step": "wait", "seconds": 5},
                 {"step": "relay_on", "device": "kmtronic8", "port": "Pi5"}], ctx)


def test_unknown_step_kind_raises():
    model = make_model()
    ctx = make_ctx(model, FakeClient())
    with pytest.raises(StepError, match="unknown step"):
        execute([{"step": "detonate"}], ctx)


def test_execute_reports_progress():
    model = make_model()
    ctx = make_ctx(model, FakeClient())
    seen = []
    execute([{"step": "wait", "seconds": 0.01}, {"step": "wait", "seconds": 0.01}],
            ctx, on_start=lambda index, total, step: seen.append((index, total)))
    assert seen == [(1, 2), (2, 2)]


def test_sleep_until_respects_deadline():
    model = make_model()
    base = {"t": 0.0}
    ctx = make_ctx(model, FakeClient())
    ctx.clock = lambda: base["t"]
    ctx._sleeper = lambda seconds: base.__setitem__("t", base["t"] + 0.1)
    assert ctx.sleep_until(base["t"] + 1.0, lambda: False) is False
    assert ctx.is_aborted() is False


def test_sleep_until_aborts():
    model = make_model()
    ctx = make_ctx(model, FakeClient())
    ctx.abort()
    with pytest.raises(StepError, match="aborted"):
        ctx.sleep_until(time.time() + 10, lambda: False)
