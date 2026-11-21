from indi_relay_gui.indi import StreamParser, vector_from_element
import xml.etree.ElementTree as ET


DEF_SWITCH = (
    '<defSwitchVector device="Relay16" name="PORTS" label="Ports" state="IPS_OK" perm="IP_RW" rule="ISR_NOFMANY">'
    '<defSwitch name="PORT01" label="Pi5">On</defSwitch>'
    '<defSwitch name="PORT02" label="none">Off</defSwitch>'
    '<defSwitch name="PORT03" label="Mount">On</defSwitch>'
    "</defSwitchVector>"
)
SET_NUMBER = (
    '<setNumberVector device="eulRoof" name="ROOF_TRAVEL" state="IPS_OK">'
    '<oneNumber name="OPEN_TIME_S" min="5" max="600" step="1">60</oneNumber>'
    "</setNumberVector>"
)
SET_TEXT = (
    '<setTextVector device="Relay16" name="BOX_HOST" state="IPS_OK">'
    '<oneText name="HOST">192.168.1.28</oneText>'
    "</setTextVector>"
)
SET_LIGHT = (
    '<setLightVector device="eulRoof" name="ROOF_SAFETY_STATUS" state="IPS_ALERT">'
    '<oneLight name="MOUNT_PARKED">IPS_OK</oneLight>'
    '<oneLight name="IMU_SAFE">IPS_ALERT</oneLight>'
    "</setLightVector>"
)


def test_parse_def_switch_vector():
    vector = vector_from_element(ET.fromstring(DEF_SWITCH))
    assert vector["device"] == "Relay16"
    assert vector["name"] == "PORTS"
    assert vector["type"] == "SwitchVector"
    assert vector["state"] == "IPS_OK"
    assert vector["rule"] == "ISR_NOFMANY"
    assert vector["members"]["PORT01"]["on"] is True
    assert vector["members"]["PORT02"]["on"] is False
    assert vector["members"]["PORT01"]["label"] == "Pi5"
    assert set(vector["members"]) == {"PORT01", "PORT02", "PORT03"}


def test_parse_number_vector():
    vector = vector_from_element(ET.fromstring(SET_NUMBER))
    member = vector["members"]["OPEN_TIME_S"]
    assert member["value"] == 60.0
    assert member["min"] == 5.0 and member["max"] == 600.0


def test_light_members_normalized():
    vector = vector_from_element(ET.fromstring(
        '<setLightVector device="EulFocuser" name="SCOPE_SAFETY" state="Ok">'
        '<oneLight name="SAFE">Ok</oneLight>'
        "</setLightVector>"))
    assert vector["members"]["SAFE"]["light"] == "IPS_OK"
    vector = vector_from_element(ET.fromstring(
        '<setLightVector device="EulFocuser" name="SCOPE_SAFETY" state="Idle">'
        '<oneLight name="SAFE">Alert</oneLight>'
        "</setLightVector>"))
    assert vector["members"]["SAFE"]["light"] == "IPS_ALERT"


def test_stream_parser_reassembles_fragments():
    received = []
    parser = StreamParser(received.append)
    payload = (DEF_SWITCH + SET_NUMBER + SET_TEXT + SET_LIGHT).encode()
    position = 0
    for size in (7, 13, 5, 40, 1, len(payload)):
        while position < len(payload):
            parser.feed(payload[position:position + size])
            position += size
    names = [v["name"] for v in received]
    assert names == ["PORTS", "ROOF_TRAVEL", "BOX_HOST", "ROOF_SAFETY_STATUS"]
    assert received[3]["members"]["IMU_SAFE"]["light"] == "IPS_ALERT"


def test_stream_parser_non_vector_elements():
    received = []
    parser = StreamParser(received.append)
    parser.feed(b'<getProperties version="1.7"/>\n'
                b'<delProperty device="Relay16" name="PORTS"/>\n'
                b'<message device="Relay16" message="hello world"/>\n'
                b'<message device="Relay16">inline message</message>\n')
    kinds = [item["type"] for item in received]
    assert kinds == ["getProperties", "delProperty", "message", "message"]
    assert received[1]["device"] == "Relay16" and received[1]["name"] == "PORTS"
    assert received[2]["message"] == "hello world"
    assert received[3]["message"] == "inline message"


