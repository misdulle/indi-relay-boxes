"""Minimal INDI XML-over-TCP client (indiserver <-> client protocol).

Discovers nothing, requires no libindi: property definitions and updates arrive
as XML fragments streamed on the socket, commands are small XML documents.
Callbacks are invoked from the reader thread; the GUI bridges them into the Qt
event loop.  Pure stdlib so the state layer stays testable without Qt.
"""

import socket
import threading


import xml.etree.ElementTree as ET
SWITCH_VECTOR = "SwitchVector"
TEXT_VECTOR = "TextVector"
NUMBER_VECTOR = "NumberVector"
LIGHT_VECTOR = "LightVector"

_LT = ord("<")
_GT = ord(">")
_QUOTES = (ord("\""), ord("'"))
_SLASH = ord("/")
_NAME_EXTRA = (ord(":"), ord("-"), ord("_"), ord("."))
_ALNUM = lambda byte: chr(byte).isalnum()

_DEF_TAGS = {
    "defSwitchVector": SWITCH_VECTOR,
    "defTextVector": TEXT_VECTOR,
    "defNumberVector": NUMBER_VECTOR,
    "defLightVector": LIGHT_VECTOR,
}
_SET_TAGS = {
    "setSwitchVector": SWITCH_VECTOR,
    "setTextVector": TEXT_VECTOR,
    "setNumberVector": NUMBER_VECTOR,
    "setLightVector": LIGHT_VECTOR,
}


def vector_from_element(elem):
    kind = _DEF_TAGS.get(elem.tag) or _SET_TAGS.get(elem.tag)
    if kind is None:
        return None
    vec = {
        "type": kind,
        "device": elem.get("device", ""),
        "name": elem.get("name", ""),
        "label": elem.get("label", ""),
        "group": elem.get("group", ""),
        "state": elem.get("state", "IPS_IDLE"),
        "perm": elem.get("perm", ""),
        "rule": elem.get("rule", ""),
        "message": elem.get("message", ""),
        "members": {},
    }
    for child in elem:
        member = {
            "name": child.get("name", ""),
            "label": child.get("label", child.get("name", "")),
        }
        if kind == SWITCH_VECTOR:
            member["on"] = (child.text or "").strip().lower() == "on"
        elif kind == TEXT_VECTOR:
            member["text"] = child.text or ""
        elif kind == NUMBER_VECTOR:
            member["value"] = float(child.text) if (child.text or "").strip() else 0.0
            for attr in ("min", "max", "step"):
                member[attr] = float(child.get(attr, 0))
        else:
            member["light"] = child.get("state") or (child.text or "").strip() or "IPS_IDLE"
        vec["members"][member["name"]] = member
    return vec


class StreamParser:
    """Splits an INDI XML stream into complete top-level elements.

    Scans the buffer manually (quote-aware) because expat cannot report the
    end of the final top-level element without an end-of-stream signal.
    """

    def __init__(self, on_element):
        self.on_element = on_element
        self._buffer = bytearray()

    def feed(self, data):
        self._buffer += data
        self._process()

    def reset(self):
        self._buffer = bytearray()

    def _process(self):
        data = self._buffer
        n = len(data)
        i = 0
        while True:
            while i < n and data[i:i + 1].isspace():
                i += 1
            if i >= n:
                break
            if data[i] != _LT:
                i += 1
                continue
            scan = _scan_tag(data, i)
            if scan is None:
                break
            name_end, closing, self_closing, gt = scan
            if closing:
                i = gt + 1
                continue
            name = data[i + 1:name_end].decode("utf-8", "replace")
            if self_closing:
                end = gt + 1
            else:
                end = _find_element_end(data, gt + 1, name)
                if end is None:
                    break
            root = _parse_root(bytes(data[i:end]))
            i = end
            self._dispatch_element(root)
        if i:
            self._buffer = data[i:]

    def _dispatch_element(self, root):
        if root.tag == "message":
            self.on_element({"type": "message", "device": root.get("device", ""),
                             "message": root.get("message") or (root.text or "").strip()})
            return
        vector = vector_from_element(root)
        if vector is not None:
            self.on_element(vector)
            return
        self.on_element({"type": root.tag, "device": root.get("device", ""),
                         "name": root.get("name", "")})

def _scan_tag(data, start):
    """Parses the markup starting at '<'; returns (name_end, closing, self_closing, gt_index)."""
    n = len(data)
    i = start + 1
    if i < n and data[i] == _SLASH:
        closing = True
        i += 1
    else:
        closing = False
    name_end = i
    while name_end < n and (_ALNUM(data[name_end]) or data[name_end] in _NAME_EXTRA):
        name_end += 1
    quote = None
    j = name_end
    while j < n:
        char = data[j]
        if quote is not None:
            if char == quote:
                quote = None
        elif char in _QUOTES:
            quote = char
        elif char == _GT:
            self_closing = data[j - 1] == _SLASH
            return name_end, closing, self_closing, j
        j += 1
    return None


def _find_element_end(data, start, name):
    """Returns the index just past the closing tag of <name>...</name>, or None."""
    n = len(data)
    i = start
    stack = [name]
    while i < n:
        if data[i] != _LT:
            i += 1
            continue
        scan = _scan_tag(data, i)
        if scan is None:
            return None
        name_end, closing, self_closing, gt = scan
        if closing:
            tag = data[i + 2:name_end].decode("utf-8", "replace")
            while stack and stack[-1] != tag:
                stack.pop()
            if stack:
                stack.pop()
            if not stack:
                return gt + 1
        elif not self_closing:
            stack.append(data[i + 1:name_end].decode("utf-8", "replace"))
        i = gt + 1
    return None


def _parse_root(chunk):
    return ET.fromstring(chunk)


class IndiClient:
    """Thin connection to one indiserver. Commands are property-set primitives."""

    def __init__(self, host, port, on_vector=None, on_del_property=None,
                 on_message=None, on_disconnected=None, connect_timeout=5.0):
        self.host = host
        self.port = port
        self.on_vector = on_vector or (lambda vector: None)
        self.on_del_property = on_del_property or (lambda device, name: None)
        self.on_message = on_message or (lambda device, message: None)
        self.on_disconnected = on_disconnected or (lambda error: None)
        self.connect_timeout = connect_timeout
        self._sock = None
        self._send_lock = threading.Lock()
        self._reader = None
        self._closing = threading.Event()

    def connect(self):
        if self.is_connected():
            return True
        self._closing.clear()
        sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        sock.settimeout(0.5)
        self._sock = sock
        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="indi-reader")
        self._reader.start()
        self.send('<getProperties version="1.7"/>\n')
        return True

    def is_connected(self):
        return self._sock is not None

    def close(self):
        self._closing.set()
        sock, self._sock = self._sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()
        reader, self._reader = self._reader, None
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=2.0)

    def _read_loop(self):
        parser = StreamParser(self._dispatch)
        while not self._closing.is_set():
            if self._sock is None:
                break
            try:
                chunk = self._sock.recv(65536)
                if not chunk:
                    break
                parser.feed(chunk)
            except socket.timeout:
                continue
            except OSError:
                break
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
        self._sock = None
        if not self._closing.is_set():
            self.on_disconnected("connection closed by server")

    def _dispatch(self, elem):
        kind = elem["type"]
        if kind in (SWITCH_VECTOR, TEXT_VECTOR, NUMBER_VECTOR, LIGHT_VECTOR):
            self.on_vector(elem)
        elif kind == "delProperty":
            self.on_del_property(elem["device"], elem["name"])
        elif kind == "message":
            self.on_message(elem["device"], elem["message"])

    def send(self, xml_text):
        sock = self._sock
        if sock is None:
            raise ConnectionError("not connected to the indiserver")
        with self._send_lock:
            sock.sendall(xml_text.encode("utf-8") + b"\n")

    def _members_xml(self, members):
        parts = []
        for name, value in members:
            if isinstance(value, str):
                parts.append("<oneText name='%s'>%s</oneText>" % (name, _xml_escape(value)))
            elif isinstance(value, bool):
                parts.append("<oneSwitch name='%s'>%s</oneSwitch>" % (name, "On" if value else "Off"))
            else:
                parts.append("<oneNumber name='%s'>%.10g</oneNumber>" % (name, value))
        return "".join(parts)

    def new_switch(self, device, name, members):
        self.send("<newSwitchVector device='%s' name='%s'>%s</newSwitchVector>"
                  % (_xml_escape(device), _xml_escape(name), self._members_xml(members)))

    def new_text(self, device, name, members):
        self.send("<newTextVector device='%s' name='%s'>%s</newTextVector>"
                  % (_xml_escape(device), _xml_escape(name), self._members_xml(members)))

    def new_number(self, device, name, members):
        self.send("<newNumberVector device='%s' name='%s'>%s</newNumberVector>"
                  % (_xml_escape(device), _xml_escape(name), self._members_xml(members)))


def _xml_escape(text):
    return (text.replace("&", "&amp;").replace("<", "&lt;")
            .replace(">", "&gt;").replace("'", "&apos;"))
