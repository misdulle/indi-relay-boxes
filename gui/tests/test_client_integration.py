import socket
import threading
import time
import xml.etree.ElementTree as ET

import pytest

from indi_relay_gui.indi import IndiClient


DEF_VECTOR = (
    '<defSwitchVector device="Relay16" name="PORTS" state="IPS_OK" perm="IP_RW" rule="ISR_NOFMANY">'
    '<defSwitch name="PORT01" label="Pi5">On</defSwitch>'
    '<defSwitch name="PORT02" label="Mount">Off</defSwitch>'
    "</defSwitchVector>"
).encode("utf-8")

SET_VECTOR = (
    '<setSwitchVector device="Relay16" name="PORTS" state="IPS_OK">'
    '<oneSwitch name="PORT01">On</oneSwitch>'
    '<oneSwitch name="PORT02">On</oneSwitch>'
    "</setSwitchVector>"
).encode("utf-8")


class FakeIndiServer:
    def __init__(self, reply=DEF_VECTOR):
        self.reply = reply
        self.received = []
        self.clients = []
        self._stop = threading.Event()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(("127.0.0.1", 0))
        self._sock.listen(1)
        self._sock.settimeout(0.2)
        self.port = self._sock.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        for client in self.clients:
            try:
                client.close()
            except OSError:
                pass
        self._sock.close()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _serve(self):
        while not self._stop.is_set():
            try:
                conn, _ = self._sock.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            conn.settimeout(0.2)
            self.clients.append(conn)
            threading.Thread(target=self._handle, args=(conn,), daemon=True).start()

    def _handle(self, conn):
        try:
            conn.sendall(self.reply + b"\n")
        except OSError:
            return
        buffer = b""
        while not self._stop.is_set():
            try:
                data = conn.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                return
            if not data:
                return
            buffer += data
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if line.strip():
                    self.received.append(line.decode("utf-8"))


def wait_until(predicate, timeout=3.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_client_discovers_properties_and_sends_commands():
    server = FakeIndiServer()
    server.start()
    received = []
    messages = []
    closed = []
    client = IndiClient("127.0.0.1", server.port,
                        on_vector=received.append,
                        on_del_property=lambda d, n: None,
                        on_message=lambda d, m: messages.append((d, m)),
                        on_disconnected=lambda reason: closed.append(reason))
    try:
        assert client.connect() is True
        assert wait_until(lambda: any("getProperties" in line for line in server.received))
        assert wait_until(lambda: len(received) == 1)
        vector = received[0]
        assert vector["device"] == "Relay16"
        assert vector["members"]["PORT01"]["on"] is True

        client.new_switch("Relay16", "PORTS", [("PORT02", True)])
        assert wait_until(lambda: any("newSwitchVector" in line for line in server.received))
        assert "<oneSwitch name='PORT02'>On</oneSwitch>" in server.received[-1]

        client.new_text("Relay16", "PORT_NAMES", [("NAME01", "Pi 5")])
        assert wait_until(lambda: any("newTextVector" in line for line in server.received))

        connection_lost = threading.Event()
        server.stop()
        assert wait_until(lambda: closed or connection_lost.is_set())
        assert client.is_connected() is False
    finally:
        client.close()
        server.stop()


def test_client_sends_xml_escaping():
    server = FakeIndiServer(reply=b"<message device='X' message='ok'/>\n")
    server.start()
    messages = []
    client = IndiClient("127.0.0.1", server.port, on_message=lambda d, m: messages.append(m))
    try:
        client.connect()
        assert wait_until(lambda: server.received)
        client.new_text("X", "PORT_NAMES", [("NAME01", "Co <d'or>")])
        assert wait_until(lambda: any("newTextVector" in line for line in server.received))
        line = server.received[-1]
        assert "Co &lt;d&apos;or&gt;" in line
        ET.fromstring(line)
    finally:
        client.close()
        server.stop()


def test_client_send_without_connection_raises():
    client = IndiClient("127.0.0.1", 1)
    with pytest.raises(ConnectionError):
        client.new_switch("X", "Y", [("Z", True)])
