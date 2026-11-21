"""Health checks evaluated while the operating mode is "running".

Two built-in kinds, both configured through YAML (`checks:` section) and easily
extendable: write a class with run_check() -> (bool, str) and register it in
RUNNERS.  A check that keeps failing for `failures` consecutive evaluations
triggers its `recovery` sequence once, followed by a grace period.
"""

import socket
import subprocess
import threading
import time

from .sequence import StepContext, StepError, execute


class PingCheck:
    kind = "ping"

    def __init__(self, spec):
        self.host = spec["host"]
        self.timeout_s = spec["timeout_s"]

    def start(self, now):
        pass

    def stop(self):
        pass

    def run_check(self, now):
        try:
            completed = subprocess.run(
                ["ping", "-c", "1", "-W", str(self.timeout_s), self.host],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=self.timeout_s + 2.0)
            if completed.returncode == 0:
                return True, "ping ok"
            return False, "ping failed (host %s unreachable)" % self.host
        except subprocess.TimeoutExpired:
            return False, "ping timed out"
        except OSError as error:
            return False, "ping could not run: %s" % error


class UdpCheck:
    kind = "udp"

    def __init__(self, spec):
        self.bind = spec.get("bind", "0.0.0.0")
        self.port = spec["port"]
        self.timeout_s = spec["timeout_s"]
        self._sock = None
        self._last_rx = None
        self._stop = threading.Event()
        self._last_message = "no datagram received yet"
        self._thread = threading.Thread(target=self._listen, daemon=True, name="udp-check")

    def start(self, now):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((self.bind, self.port))
        except OSError as error:
            self._last_message = "cannot bind UDP %s:%d: %s" % (self.bind, self.port, error)
            self._sock = None
            return
        self._last_rx = None
        self._stop.clear()
        self._thread.start()

    def stop(self):
        self._stop.set()
        sock, self._sock = self._sock, None
        if sock is not None:
            sock.close()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _listen(self):
        while not self._stop.is_set():
            sock = self._sock
            if sock is None:
                return
            try:
                sock.settimeout(0.5)
                data, addr = sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                return
            if data:
                self._last_rx = time.time()
                self._last_message = "last datagram %ss ago from %s (%d bytes)" % (
                    "%.0f" % max(0, time.time() - self._last_rx), addr[0], len(data))

    def run_check(self, now):
        if self._sock is None:
            return False, self._last_message
        if self._last_rx is None:
            return False, "no datagram received on UDP port %d yet" % self.port
        age = now - self._last_rx
        if age > self.timeout_s:
            return False, self._last_message
        return True, self._last_message


RUNNERS = {"ping": PingCheck, "udp": UdpCheck}


def build_checks(check_cfg):
    instances = []
    for spec in check_cfg.get("items", []):
        factory = RUNNERS.get(spec["kind"])
        if factory is not None:
            instances.append(factory(spec))
    return instances


class CheckRunner:
    """Evaluates every enabled check each interval; reacts with recovery sequences."""

    def __init__(self, check_cfg, sequence_steps_for, ctx_factory, log,
                 clock=time.time, interval_s=None, grace_s=None):
        self.cfg = check_cfg
        self.interval_s = interval_s if interval_s is not None else check_cfg["interval_s"]
        self.grace_s = grace_s if grace_s is not None else check_cfg["grace_s"]
        self.sequence_steps_for = sequence_steps_for
        self.ctx_factory = ctx_factory
        self.log = log
        self.clock = clock
        self.statuses = {}
        for spec in check_cfg.get("items", []):
            self.statuses[spec["name"]] = {"failures": 0, "recovering": False,
                                           "last_ok": None, "last_message": "not evaluated yet",
                                           "last_change": None, "recoveries": 0}
        self._specs = {spec["name"]: spec for spec in check_cfg.get("items", [])
                       if spec.get("enabled", True)}
        self._instances = {}
        self._blocked_until = 0.0
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="check-runner")

    def start(self):
        if self._thread.is_alive():
            return
        now = self.clock()
        for name, spec in self._specs.items():
            instance = RUNNERS[spec["kind"]](spec)
            instance.start(now)
            self._instances[name] = instance
        self._stop.clear()
        self._thread.start()

    def stop(self, join_timeout=3.0):
        self._stop.set()
        for instance in self._instances.values():
            instance.stop()
        if self._thread.is_alive():
            self._thread.join(timeout=join_timeout)

    def status_snapshot(self):
        return {name: dict(entry) for name, entry in self.statuses.items()}

    def _loop(self):
        next_run = self.clock()
        while not self._stop.is_set():
            self._stop.wait(0.5)
            if self._stop.is_set():
                return
            now = self.clock()
            if now < next_run or self._recovering():
                if now >= next_run:
                    next_run = now + self.interval_s
                continue
            next_run = now + self.interval_s
            self.evaluate(now)

    def _recovering(self):
        return any(entry["recovering"] for entry in self.statuses.values())

    def evaluate(self, now, override_specs=None):
        results = {}
        for name, spec in self._specs.items():
            if override_specs is not None and name not in override_specs:
                continue
            instance = self._instances.get(name)
            if instance is None:
                continue
            ok, message = instance.run_check(now)
            results[name] = self._record(name, spec, ok, message, now)
        self._react(results, now)
        return results

    def _record(self, name, spec, ok, message, now):
        entry = self.statuses[name]
        changed = entry["last_ok"] != ok
        entry["last_ok"] = ok
        entry["last_message"] = message
        entry["last_change"] = now if changed else entry["last_change"]
        if ok:
            entry["failures"] = 0
            entry["recovering"] = False
        else:
            entry["failures"] += 1
        if changed:
            self.log("check %s: %s (%s)" % (name, "OK" if ok else "FAIL", message))
        return entry

    def _react(self, results, now):
        for name, entry in results.items():
            failures_needed = self._specs[name].get("failures", 3)
            if entry["last_ok"] or entry["recovering"] or entry["failures"] < failures_needed:
                continue
            if self._blocked_until and self.clock() < self._blocked_until:
                continue
            steps = self.sequence_steps_for(name)
            if not steps:
                entry["recovering"] = True
                self.log("check %s: recovery placeholder failed" % name)
                entry["recovering"] = False
                continue
            entry["recovering"] = True
            self.log("check %s: %d failure(s) -> running recovery sequence" % (name, entry["failures"]))
            try:
                ctx = self.ctx_factory()
                execute(steps, ctx)
                entry["recoveries"] += 1
                self.log("check %s: recovery done" % name)
            except StepError as error:
                self.log("check %s: recovery FAILED: %s" % (name, error))
            finally:
                entry["recovering"] = False
                entry["failures"] = 0
                self._blocked_until = self.clock() + self.grace_s


def make_ctx_factory(model, client, devices, log, roof_timeout_s, clock=time.time):
    def factory():
        ctx = StepContext(model, client, devices, log=log,
                          abort_event=None, roof_timeout_s=roof_timeout_s, clock=clock)
        ctx.roof_state = lambda device: (model.roof_state(device).get("state") or "unknown")
        return ctx

    return factory
