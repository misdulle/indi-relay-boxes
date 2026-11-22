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


def test_set_update_keeps_labels_and_merges_members():
    model = make_model()
    assert [p["label"] for p in model.ports("KMTronic8")] == ["Pi5", "Mount", "camera"]
    update = vector_from_element(ET.fromstring(
        '<setSwitchVector device="KMTronic8" name="PORTS" state="IPS_BUSY">'
        '<oneSwitch name="PORT01">Off</oneSwitch>'
        '<oneSwitch name="PORT02">On</oneSwitch>'
        '<oneSwitch name="PORT03">On</oneSwitch>'
        '<oneSwitch name="PORT04">Off</oneSwitch>'
        "</setSwitchVector>"))
    model.apply_vector(update)
    ports = model.ports("KMTronic8")
    assert [p["label"] for p in ports] == ["Pi5", "Mount", "camera"]
    assert [p["on"] for p in ports] == [False, True, False]
    assert model.get("KMTronic8", "PORTS")["state"] == "IPS_BUSY"


def test_set_update_keeps_new_member_and_defines_wins():
    model = make_model()
    update = vector_from_element(ET.fromstring(
        '<setSwitchVector device="KMTronic8" name="PORTS" state="IPS_OK">'
        '<oneSwitch name="PORT05">On</oneSwitch>'
        "</setSwitchVector>"))
    model.apply_vector(update)
    ports = {p["name"]: p for p in model.ports("KMTronic8")}
    assert ports["PORT05"]["on"] is True

    definition = vector_from_element(ET.fromstring(
        '<defSwitchVector device="KMTronic8" name="PORTS" state="IPS_OK">'
        '<defSwitch name="PORT01" label="Renamed">On</defSwitch>'
        "</defSwitchVector>"))
    model.apply_vector(definition)
    labels = [p["label"] for p in model.ports("KMTronic8")]
    assert labels == ["Renamed"]

    update = vector_from_element(ET.fromstring(
        '<setSwitchVector device="KMTronic8" name="PORTS" state="IPS_OK">'
        '<oneSwitch name="PORT01">On</oneSwitch>'
        '<oneSwitch name="PORT05">Off</oneSwitch>'
        "</setSwitchVector>"))
    model.apply_vector(update)
    names_ = {p["name"]: p["on"] for p in model.ports("KMTronic8")}
    assert names_ == {"PORT01": True, "PORT05": False}


def test_set_update_light_values_only():
    model = make_model()
    update = vector_from_element(ET.fromstring(
        '<setLightVector device="eulRoof" name="ROOF_SAFETY_STATUS" state="IPS_OK">'
        '<oneLight name="IMU_SAFE">Ok</oneLight>'
        "</setLightVector>"))
    model.apply_vector(update)
    lights = model.roof_safety("eulRoof")
    assert lights == {"MOUNT_PARKED": "IPS_OK", "IMU_SAFE": "IPS_OK"}


def mount_coord_vector():
    return vector_from_element(ET.fromstring(
        '<setNumberVector device="EQMod Mount" name="EQUATORIAL_EOD_COORD" state="IPS_OK">'
        '<oneNumber name="RA">18.5</oneNumber>'
        '<oneNumber name="DEC">38.7939</oneNumber>'
        "</setNumberVector>"))


def pier_vector(side="west"):
    return vector_from_element(ET.fromstring(
        '<defSwitchVector device="EQMod Mount" name="TELESCOPE_PIER_SIDE" state="IPS_OK" rule="ISR_1OFMANY">'
        '<defSwitch name="PIER_EAST">%s</defSwitch>'
        '<defSwitch name="PIER_WEST">%s</defSwitch>'
        "</defSwitchVector>" % ("On" if side == "east" else "Off",
                                "On" if side == "west" else "Off")))


def park_vector(parked=True):
    return vector_from_element(ET.fromstring(
        '<defSwitchVector device="EQMod Mount" name="TELESCOPE_PARK" state="IPS_OK">'
        '<defSwitch name="PARK">%s</defSwitch>'
        '<defSwitch name="UNPARK">Off</defSwitch>'
        "</defSwitchVector>" % ("On" if parked else "Off")))


def site_vector():
    return vector_from_element(ET.fromstring(
        '<defNumberVector device="EQMod Mount" name="GEOGRAPHIC_COORD" state="IPS_OK">'
        '<oneNumber name="LATITUDE">48.2</oneNumber>'
        '<oneNumber name="LONGITUDE">7.6</oneNumber>'
        "</defNumberVector>"))


def test_mount_state_reads_pointing_pier_and_park():
    model = IndiModel()
    assert model.mount_state("EQMod Mount") == {"ra": None, "dec": None, "lst": None,
                                                "az": None, "alt": None,
                                                "pier": None, "parked": None,
                                                "online": False}
    model.apply_vector(mount_coord_vector())
    model.apply_vector(pier_vector("west"))
    model.apply_vector(park_vector(True))
    state = model.mount_state("EQMod Mount")
    assert state["ra"] == 18.5
    assert state["dec"] == 38.7939
    assert state["pier"] == "west"
    assert state["parked"] is True
    assert state["online"] is True


def test_mount_state_park_switch_reports_unparked():
    model = IndiModel()
    model.apply_vector(park_vector(False))
    assert model.mount_state("EQMod Mount")["parked"] is False


def horizontal_vector():
    return vector_from_element(ET.fromstring(
        '<defNumberVector device="EQMod Mount" name="HORIZONTAL_COORD" state="IPS_OK">'
        '<oneNumber name="AZ">212.5</oneNumber>'
        '<oneNumber name="ALT">33.75</oneNumber>'
        "</defNumberVector>"))


def test_mount_state_reads_horizontal_coord():
    model = IndiModel()
    model.apply_vector(horizontal_vector())
    state = model.mount_state("EQMod Mount")
    assert state["az"] == 212.5
    assert state["alt"] == 33.75
    assert state["online"] is True


def lst_vector(value="18.75"):
    return vector_from_element(ET.fromstring(
        '<defNumberVector device="EQMod Mount" name="TIME_LST" state="IPS_OK">'
        '<oneNumber name="LST">%s</oneNumber>'
        "</defNumberVector>" % value))


def test_mount_state_reads_local_sidereal_time():
    model = IndiModel()
    assert model.mount_state("EQMod Mount")["lst"] is None
    model.apply_vector(lst_vector())
    state = model.mount_state("EQMod Mount")
    assert state["lst"] == 18.75
    assert state["online"] is True


def test_hours_accepts_plain_and_colon_notation():
    from indi_relay_gui.model import _hours
    assert _hours(None) is None
    assert _hours(18.75) == 18.75
    assert _hours("18:45:00") == 18.75
    assert _hours("-1:30") == -1.5
    assert _hours("bogus") is None


def test_site_coords():
    model = IndiModel()
    assert model.site_coords("EQMod Mount") is None
    model.apply_vector(site_vector())
    assert model.site_coords("EQMod Mount") == {"lat": 48.2, "lon": 7.6}


def relay16_vector():
    members = "".join(
        '<defSwitch name="PORT%02d" label="R%02d">Off</defSwitch>' % (n, n)
        for n in range(1, 6))
    return vector_from_element(ET.fromstring(
        '<defSwitchVector device="Relay16" name="PORTS" state="IPS_OK">'
        + members + "</defSwitchVector>"))


def roof_relay_params_vector(stop="1", open_="2", close="3"):
    return vector_from_element(ET.fromstring(
        '<defNumberVector device="eulRoof" name="ROOF_RELAY_PARAMS" state="IPS_OK">'
        '<oneNumber name="TCP_PORT">80</oneNumber>'
        '<oneNumber name="ADDR">30</oneNumber>'
        '<oneNumber name="TIMEOUT_MS">2000</oneNumber>'
        '<oneNumber name="RELAY_STOP">%s</oneNumber>'
        '<oneNumber name="RELAY_OPEN">%s</oneNumber>'
        '<oneNumber name="RELAY_CLOSE">%s</oneNumber>'
        '<oneNumber name="PULSE_MS">500</oneNumber>'
        "</defNumberVector>" % (stop, open_, close)))


def test_roof_control_ports_from_driver_params():
    model = make_model()
    assert model.roof_control_ports("eulRoof") == set()
    model.apply_vector(roof_relay_params_vector())
    assert model.roof_control_ports("eulRoof") == {1, 2, 3}
    assert model.roof_control_ports("Relay16") == set()


def test_ports_exclude_roof_control_relays():
    model = make_model()
    model.apply_vector(relay16_vector())
    model.apply_vector(roof_relay_params_vector())
    assert [p["name"] for p in model.ports("Relay16")] == ["PORT01", "PORT02",
                                                           "PORT03", "PORT04", "PORT05"]
    filtered = model.ports("Relay16", exclude_ports=model.roof_control_ports("eulRoof"))
    assert [p["name"] for p in filtered] == ["PORT04", "PORT05"]
    assert model.ports("Relay16", exclude_ports=set()) == model.ports("Relay16")


def test_ports_exclude_ignores_non_port_members():
    model = make_model()
    model.apply_vector(relay16_vector())
    assert model.ports("Relay16", exclude_ports={99}) == model.ports("Relay16")
