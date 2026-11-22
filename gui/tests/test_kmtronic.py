import base64
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest

from indi_relay_gui import kmtronic


class FakeBox:
    def __init__(self, ff_style=True, user="", password=""):
        self.ff_style = ff_style
        self.user = user
        self.password = password
        self.states = {n: 0 for n in range(1, 9)}
        self.requests = []
        self.lock = threading.Lock()

    def status_xml(self):
        parts = ["<relay%d>%d</relay%d>" % (n, state, n)
                 for n, state in self.states.items()]
        return "<data>%s<relay0>0</relay0></data>" % "".join(parts)


def make_handler(box):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            box.requests.append(self.path)
            if box.user:
                token = base64.b64encode(("%s:%s" % (box.user, box.password)).encode()).decode()
                if self.headers.get("Authorization") != "Basic " + token:
                    self.send_response(401)
                    self.end_headers()
                    return
            path = self.path
            if path == "/status.xml":
                self._reply(200, box.status_xml())
                return
            if path.startswith("/FF"):
                if not box.ff_style:
                    self._reply(404)
                    return
                relay, on = int(path[3:5]), path[6] == "1"
                with box.lock:
                    box.states[relay] = 1 if on else 0
                self._reply(200)
                return
            if path.startswith("/relays.cgi"):
                relay = int(path.rsplit("=", 1)[1])
                with box.lock:
                    box.states[relay] ^= 1
                self._reply(200)
                return
            self._reply(404)

        def _reply(self, code, body=""):
            self.send_response(code)
            self.end_headers()
            if body:
                self.wfile.write(body.encode())

    return Handler


@pytest.fixture
def box():
    return FakeBox()


@pytest.fixture
def serve(box):
    servers = []

    def start(ff_style=True, user="", password=""):
        box.ff_style = ff_style
        box.user = user
        box.password = password
        server = ThreadingHTTPServer(("127.0.0.1", 0), make_handler(box))
        servers.append(server)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        return server.server_address[1]

    yield start
    for server in servers:
        server.shutdown()
        server.server_close()


def test_http_get_serves_body(box, serve):
    port = serve()
    status, body = kmtronic.http_get("127.0.0.1", port, "/status.xml")
    assert status == 200
    assert "<relay1>0</relay1>" in body


def test_http_get_transport_failure():
    status, body = kmtronic.http_get("127.0.0.1", 1, "/status.xml", timeout_s=0.5)
    assert (status, body) == (0, "")


def test_http_get_with_auth(box, serve):
    port = serve(user="admin", password="secret")
    status, _ = kmtronic.http_get("127.0.0.1", port, "/status.xml")
    assert status == 401
    status, body = kmtronic.http_get("127.0.0.1", port, "/status.xml",
                                     "admin", "secret")
    assert status == 200
    assert "<relay1>0</relay1>" in body


def test_read_relay_parses_state(box, serve):
    port = serve()
    assert kmtronic.read_relay("127.0.0.1", port, "", "", 1) is False
    with box.lock:
        box.states[1] = 1
    assert kmtronic.read_relay("127.0.0.1", port, "", "", 1) is True


def test_read_relay_missing_relay_raises(box, serve):
    port = serve()
    with pytest.raises(kmtronic.KmtronicError):
        kmtronic.read_relay("127.0.0.1", port, "", "", 9)


def test_read_relay_on_http_error(box, serve):
    port = serve(user="admin", password="secret")
    with pytest.raises(kmtronic.KmtronicError):
        kmtronic.read_relay("127.0.0.1", port, "", "", 1)


def test_set_relay_ff_style(box, serve):
    port = serve()
    result = kmtronic.set_relay("127.0.0.1", port, "", "", 1, True)
    assert result == ("/FF0101", False)
    with box.lock:
        assert box.states[1] == 1
    result = kmtronic.set_relay("127.0.0.1", port, "", "", 1, False)
    assert result == ("/FF0100", False)
    with box.lock:
        assert box.states[1] == 0


def test_set_relay_ff_style_rejects_http_error(box, serve):
    port = serve(user="admin", password="secret")
    with pytest.raises(kmtronic.KmtronicError):
        kmtronic.set_relay("127.0.0.1", port, "", "", 1, True)


def test_set_relay_cgi_fallback(box, serve):
    port = serve(ff_style=False)
    assert kmtronic.set_relay("127.0.0.1", port, "", "", 2, True) == ("/relays.cgi?relay=2", False)
    with box.lock:
        assert box.states[2] == 1
    assert box.requests == ["/FF0201", "/status.xml", "/relays.cgi?relay=2"]


def test_set_relay_cgi_fallback_skips_when_already_on(box, serve):
    port = serve(ff_style=False)
    with box.lock:
        box.states[1] = 1
    result = kmtronic.set_relay("127.0.0.1", port, "", "", 1, True)
    assert result == ("/status.xml (already on)", True)
    assert box.requests == ["/FF0101", "/status.xml"]


def test_set_relay_cgi_fallback_skips_when_already_off(box, serve):
    port = serve(ff_style=False)
    result = kmtronic.set_relay("127.0.0.1", port, "", "", 1, False)
    assert result == ("/status.xml (already off)", True)
    assert box.requests == ["/FF0100", "/status.xml"]


def test_power_cycles_existing_relay(box, serve):
    port = serve()
    with box.lock:
        box.states[1] = 1
    path, cycled = kmtronic.power_on("127.0.0.1", port, "", "", 1, pause_s=0)
    assert cycled is True
    assert path == "/FF0100, then /FF0101"
    with box.lock:
        assert box.states[1] == 1
    assert box.requests == ["/status.xml", "/FF0100", "/FF0101"]


def test_power_on_off_relay(box, serve):
    port = serve()
    path, cycled = kmtronic.power_on("127.0.0.1", port, "", "", 1, pause_s=0)
    assert (path, cycled) == ("/FF0101", False)
    with box.lock:
        assert box.states[1] == 1
    assert box.requests == ["/status.xml", "/FF0101"]


def test_power_on_cycles_via_cgi(box, serve):
    port = serve(ff_style=False)
    with box.lock:
        box.states[1] = 1
    path, cycled = kmtronic.power_on("127.0.0.1", port, "", "", 1, pause_s=0)
    assert (path, cycled) == ("/relays.cgi?relay=1, then /relays.cgi?relay=1", True)
    with box.lock:
        assert box.states[1] == 1


def test_power_on_unknown_state_switches_on(monkeypatch, box, serve):
    port = serve()

    def broken(*args, **kwargs):
        raise kmtronic.KmtronicError("no status.xml")

    monkeypatch.setattr(kmtronic, "read_relay", broken)
    path, cycled = kmtronic.power_on("127.0.0.1", port, "", "", 1, pause_s=0)
    assert (path, cycled) == ("/FF0101", False)
    with box.lock:
        assert box.states[1] == 1


def test_set_relay_forced_cgi(box, serve):
    port = serve(ff_style=True)
    result = kmtronic.set_relay("127.0.0.1", port, "", "", 3, True, force_cgi=True)
    assert result == ("/relays.cgi?relay=3", False)
    with box.lock:
        assert box.states[3] == 1


@pytest.mark.parametrize("code,expected", [(0, True), (1, False)])
def test_host_reachable_uses_ping_returncode(monkeypatch, code, expected):
    class Completed:
        returncode = code

    monkeypatch.setattr(kmtronic.subprocess, "run", lambda *a, **k: Completed())
    assert kmtronic.host_reachable("192.168.1.29") is expected


def test_host_reachable_false_when_ping_missing(monkeypatch):
    def boom(*args, **kwargs):
        raise FileNotFoundError("no ping")

    monkeypatch.setattr(kmtronic.subprocess, "run", boom)
    assert kmtronic.host_reachable("192.168.1.29") is False


def test_host_reachable_false_on_timeout(monkeypatch):
    def slow(*args, **kwargs):
        raise kmtronic.subprocess.TimeoutExpired(cmd="ping", timeout=1)

    monkeypatch.setattr(kmtronic.subprocess, "run", slow)
    assert kmtronic.host_reachable("192.168.1.29") is False
