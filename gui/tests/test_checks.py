import socket
import subprocess
import threading
import time


from indi_relay_gui import checks as checks_mod
from indi_relay_gui.checks import CheckRunner, PingCheck, UdpCheck, build_checks
from indi_relay_gui.model import IndiModel


class FakeClient:
    def __init__(self):
        self.switches = []

    def new_switch(self, device, name, members):
        self.switches.append((device, name, list(members)))


def make_ping_spec(**overrides):
    spec = {"name": "pi5_ping", "kind": "ping", "host": "192.0.2.10",
            "timeout_s": 1, "failures": 2, "enabled": True, "recovery": []}
    spec.update(overrides)
    return spec


def test_ping_check_ok_and_fail(monkeypatch):
    check = PingCheck({"host": "127.0.0.1", "timeout_s": 2})
    monkeypatch.setattr(checks_mod.subprocess, "run",
                        lambda *args, **kwargs: subprocess.CompletedProcess([], 0))
    assert check.run_check(time.time()) == (True, "ping ok")
    monkeypatch.setattr(checks_mod.subprocess, "run",
                        lambda *args, **kwargs: subprocess.CompletedProcess([], 1))
    ok, message = check.run_check(time.time())
    assert ok is False
    assert "unreachable" in message


def test_ping_check_timeout(monkeypatch):
    check = PingCheck({"host": "127.0.0.1", "timeout_s": 1})

    def slow(*args, **kwargs):
        raise subprocess.TimeoutExpired(cmd="ping", timeout=1)

    monkeypatch.setattr(checks_mod.subprocess, "run", slow)
    ok, message = check.run_check(time.time())
    assert ok is False and "timed out" in message


def test_udp_check_detects_datagrams():
    spec = {"bind": "127.0.0.1", "port": 45679, "timeout_s": 60}
    check = UdpCheck(spec)
    check.start(time.time())
    try:
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        deadline = time.time() + 3
        while check._last_rx is None and time.time() < deadline:
            sender.sendto(b"imu-data", ("127.0.0.1", 45679))
            time.sleep(0.05)
        sender.close()
        assert check._last_rx is not None
        ok, _ = check.run_check(time.time())
        assert ok is True
        check._last_rx = time.time() - 120
        ok, _ = check.run_check(time.time())
        assert ok is False
    finally:
        check.stop()


def test_udp_check_without_datagrams_fails():
    spec = {"bind": "127.0.0.1", "port": 45680, "timeout_s": 60}
    check = UdpCheck(spec)
    check.start(time.time())
    try:
        ok, message = check.run_check(time.time())
        assert ok is False and "no datagram" in message
    finally:
        check.stop()


def test_build_checks_skips_unknown_kinds():
    cfg = {"interval_s": 1, "grace_s": 0, "items": [
        make_ping_spec(), {"name": "x", "kind": "snmp", "enabled": True}]}
    instances = build_checks(cfg)
    assert len(instances) == 1
    assert isinstance(instances[0], PingCheck)


class FakeSequenceContext:
    def __init__(self, client):
        self.model = IndiModel()
        self.client = client
        self.devices = {"kmtronic8": "KMTronic8"}
        self.log = lambda *_: None
        self.roof_timeout_s = 60
        self.clock = time.time
        self.abort_event = threading.Event()

    def run_step(self, step):
        assert step["step"] == "relay_cycle"
        self.client.new_switch("KMTronic8", "PORTS", [("PORT03", False)])
        self.client.new_switch("KMTronic8", "PORTS", [("PORT03", True)])

    def is_aborted(self):
        return False


def test_check_runner_triggers_recovery_once(monkeypatch):
    client = FakeClient()
    log = []
    recovery = [{"step": "relay_cycle", "device": "kmtronic8", "port": 3, "seconds": 1}]
    cfg = {"interval_s": 1, "grace_s": 0, "items": [make_ping_spec(recovery=recovery)]}
    runner = CheckRunner(cfg,
                         sequence_steps_for=lambda name: recovery,
                         ctx_factory=lambda: FakeSequenceContext(client),
                         log=log.append,
                         clock=time.time,
                         interval_s=60, grace_s=0)
    runner._instances["pi5_ping"] = PingCheck({"host": "192.0.2.10", "timeout_s": 1})

    monkeypatch.setattr(checks_mod.subprocess, "run",
                        lambda *a, **k: subprocess.CompletedProcess([], 0))
    entry = runner.evaluate(time.time(), {"pi5_ping"})["pi5_ping"]
    assert entry["last_ok"] is True
    assert client.switches == []

    monkeypatch.setattr(checks_mod.subprocess, "run",
                        lambda *a, **k: subprocess.CompletedProcess([], 1))
    entry = runner.evaluate(time.time(), {"pi5_ping"})["pi5_ping"]
    assert entry["last_ok"] is False and entry["failures"] == 1 and client.switches == []
    entry = runner.evaluate(time.time() + 1, {"pi5_ping"})["pi5_ping"]
    assert len(client.switches) == 2
    assert entry["recoveries"] == 1 and entry["failures"] == 0
    assert entry["recovering"] is False

    entry = runner.evaluate(time.time() + 2, {"pi5_ping"})["pi5_ping"]
    assert len(client.switches) == 2 and entry["failures"] == 1
    runner.stop()
