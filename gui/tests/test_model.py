import xml.etree.ElementTree as ET

from indi_relay_gui.indi import vector_from_element
from indi_relay_gui.model import IndiModel


def port_vector():
    members = "".join(
        '<defSwitch name="PORT%02d" label="%s">%s</defSwitch>' % (index + 1, label, state)
        for index, (label, state) in enumerate(
            [("Pi5", "On"), ("none", "Off"), ("Mount", "Off"), ("camera", "On")]))
    return vector_from_element(ET.fromstring(
        '<defSwitchVector device="KMTronic8" name="PORTS" state="IPS_OK">'
        + members + "</defSwitchVector>"))


def names_vector():
    members = '<oneText name="NAME01">Pi5</oneText><oneText name="NAME02">none</oneText>'
    return vector_from_element(ET.fromstring(
        '<defTextVector device="KMTronic8" name="PORT_NAMES">' + members + "</defTextVector>"))


def roof_vector(state="IPS_OK", open_on="On", close_on="Off"):
    return vector_from_element(ET.fromstring(
        '<defSwitchVector device="eulRoof" name="DOME_SHUTTER" state="%s">'
        '<defSwitch name="SHUTTER_OPEN">%s</defSwitch>'
        '<defSwitch name="SHUTTER_CLOSE">%s</defSwitch>'
        "</defSwitchVector>" % (state, open_on, close_on)))


def safety_vector():
    return vector_from_element(ET.fromstring(
        '<defLightVector device="eulRoof" name="ROOF_SAFETY_STATUS" state="IPS_ALERT">'
        '<defLight name="MOUNT_PARKED">IPS_OK</defLight>'
        '<defLight name="IMU_SAFE">IPS_ALERT</defLight>'
        "</defLightVector>"))


def make_model():
    model = IndiModel()
    model.apply_vector(port_vector())
    model.apply_vector(names_vector())
    model.apply_vector(roof_vector())
    model.apply_vector(safety_vector())
    return model


def test_relay_devices():
    model = make_model()
    assert model.relay_devices() == ["KMTronic8"]


def test_ports_hidden_and_sorted():
    model = make_model()
    ports = model.ports("KMTronic8")
    assert [p["label"] for p in ports] == ["Pi5", "Mount", "camera"]
    assert ports[0]["on"] is True and ports[1]["on"] is False


def test_port_member_lookup():
    model = make_model()
    assert model.port_member("KMTronic8", "Pi5") == "PORT01"
    assert model.port_member("KMTronic8", "pi5") == "PORT01"
    assert model.port_member("KMTronic8", "CAMERA") == "PORT04"
    assert model.port_member("KMTronic8", 3) == "PORT03"
    assert model.port_member("KMTronic8", " nonexistent ") is None


def test_roof_state_transitions():
    model = make_model()
    assert model.roof_state("eulRoof")["state"] == "open"
    model.apply_vector(roof_vector(open_on="Off", close_on="On"))
    assert model.roof_state("eulRoof")["state"] == "closed"
    model.apply_vector(roof_vector(state="IPS_BUSY"))
    state = model.roof_state("eulRoof")
    assert state["state"] == "moving" and state["busy"] is True
    model.apply_vector(roof_vector(open_on="Off", close_on="Off", state="IPS_ALERT"))
    assert model.roof_state("eulRoof")["state"] == "error"
    model.apply_vector(roof_vector(open_on="Off", close_on="Off"))
    assert model.roof_state("eulRoof")["state"] == "unknown"
    assert model.roof_state("nowhere")["state"] is None


def test_roof_safety():
    model = make_model()
    lights = model.roof_safety("eulRoof")
    assert lights == {"MOUNT_PARKED": "IPS_OK", "IMU_SAFE": "IPS_ALERT"}
    assert model.roof_safety("nowhere") == {}


def test_port_names():
    model = make_model()
    assert model.port_names("KMTronic8")["NAME01"] == "Pi5"


def test_host_settings():
    model = make_model()
    model.apply_vector(vector_from_element(ET.fromstring(
        '<defTextVector device="KMTronic8" name="BOX_HOST">'
        '<oneText name="HOST">192.168.1.23</oneText>'
        "</defTextVector>")))
    model.apply_vector(vector_from_element(ET.fromstring(
        '<defNumberVector device="KMTronic8" name="BOX_NETWORK">'
        '<oneNumber name="TCP_PORT" min="1" max="65535" step="1">80</oneNumber>'
        '<oneNumber name="ADDR" min="0" max="99" step="1">30</oneNumber>'
        "</defNumberVector>")))
    assert model.host_settings("KMTronic8") == {"HOST": "192.168.1.23",
                                                "TCP_PORT": "80", "ADDR": "30"}


def test_config_summary_mentions_devices_and_relays():
    model = make_model()
    text = "\n".join(model.config_summary())
    assert "KMTronic8" in text
    assert "Pi5: ON" in text
    assert "eulRoof (roof): open" in text
    assert "none" not in text


def test_remove_property():
    model = make_model()
    assert model.get("KMTronic8", "PORTS") is not None
    model.remove("KMTronic8", "PORTS")
    assert model.get("KMTronic8", "PORTS") is None
    model.remove("KMTronic8", "PORTS")
