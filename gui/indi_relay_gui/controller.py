"""Operating-mode state machine for the observatory.

Modes:
  INACTIVE  display state, no relay / roof interaction (only mode changes)
  STATIC    relays and roof freely operable
  STARTING  the starting sequence runs; success switches to RUNNING
  RUNNING   configured health checks run and self-heal
  STOPPING  the stopping sequence runs; success switches to INACTIVE
"""

import threading

from .checks import CheckRunner, build_monitors
from .sequence import SequenceRunner, StepContext


class Mode:
    INACTIVE = "inactive"
    STATIC = "static"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"

    ALL = (INACTIVE, STATIC, STARTING, RUNNING, STOPPING)


class Controller:
    def __init__(self, model, client, config, log):
        self.model = model
        self.client = client
        self.config = config
        self.log = log
        self.listeners = []
        self.mode = Mode.INACTIVE
        self.runner = None
        self._abort_event = threading.Event()
        self._guard = threading.Lock()
        self.checks = None
        self.monitors = build_monitors(config["checks"])
        self.connected_devices = set()
        for monitor in self.monitors.values():
            monitor.start()

    def ensure_device_connected(self, device):
        """Send CONNECTION/CONNECT to each configured driver the first time the
        indiserver reports one of its properties. getProperties alone only
        lists the drivers; every device must be switched on explicitly."""
        if (device not in self.config["devices"].values()
                or device in self.connected_devices):
            return
        self.connected_devices.add(device)
        try:
            self.client.new_switch(device, "CONNECTION", [("CONNECT", True)])
            self.log("%s: connecting" % device)
        except Exception as error:
            self.log("%s: connect failed: %s" % (device, error))
            self.connected_devices.discard(device)

    def forget_device(self, device, name="CONNECTION"):
        if name == "CONNECTION":
            self.connected_devices.discard(device)

    def shutdown(self):
        self._stop_checks()
        for monitor in self.monitors.values():
            monitor.stop()

    def scope_safe(self):
        """True when the EulFocuser driver reports SCOPE_SAFETY/SAFE ok.
        No data (stale or offline focuser) counts as NOT safe."""
        focuser = self.config["devices"].get("focuser")
        if focuser:
            vector = self.model.get(focuser, "SCOPE_SAFETY")
            if vector is not None:
                return vector["members"].get("SAFE", {}).get("light") == "IPS_OK"
        roof = self.config["devices"].get("roof")
        vector = self.model.get(roof, "ROOF_SAFETY_STATUS")
        if vector is not None:
            return vector["members"].get("IMU_SAFE", {}).get("light") == "IPS_OK"
        return False

    def add_listener(self, listener):
        self.listeners.append(listener)

    def _emit(self, event, payload=None):
        for listener in list(self.listeners):
            listener(event, payload)

    def is_busy(self):
        return self.mode in (Mode.STARTING, Mode.STOPPING)

    def relay_actions_allowed(self):
        return self.mode == Mode.STATIC

    def roof_move_allowed(self):
        return self.mode == Mode.STATIC

    def roof_stop_allowed(self):
        return self.mode != Mode.INACTIVE

    def allowed_mode_changes(self):
        if self.is_busy():
            return []
        if self.mode == Mode.INACTIVE:
            return [Mode.STATIC, Mode.STARTING]
        if self.mode == Mode.STATIC:
            return [Mode.INACTIVE, Mode.STARTING, Mode.STOPPING]
        if self.mode == Mode.RUNNING:
            return [Mode.STOPPING]
        if self.mode in (Mode.STARTING, Mode.STOPPING):
            return []
        return []

    def set_mode(self, target):
        if target not in Mode.ALL:
            self._emit("mode_rejected", "unknown mode %r" % target)
            return False
        if target == self.mode and target not in (Mode.STARTING, Mode.STOPPING):
            self._emit("mode_changed", self.mode)
            return True
        with self._guard:
            if self.is_busy():
                self._emit("mode_rejected", "a sequence is still running (use Abort)")
                return False
            allowed = self.allowed_mode_changes()
            if target not in allowed and not (self.mode == Mode.INACTIVE and target in (Mode.STATIC, Mode.STARTING)):
                self._emit("mode_rejected", "cannot go from %s to %s" % (self.mode, target))
                return False
            if target == Mode.STARTING:
                self._enter_sequence(Mode.STARTING,
                                     self.config["sequences"]["starting"] or None,
                                     "starting", Mode.RUNNING)
                return True
            if target == Mode.STOPPING:
                self._enter_sequence(Mode.STOPPING,
                                     self.config["sequences"]["stopping"] or None,
                                     "stopping", Mode.INACTIVE)
                return True
            self._set_mode_plain(target)
            return True

    def _set_mode_plain(self, mode):
        self._stop_checks()
        self.mode = mode
        self.log("operating mode: %s" % mode)
        self._emit("mode_changed", mode)

    def _enter_sequence(self, mode, steps, key, on_success):
        self.mode = mode
        self._emit("mode_changed", mode)
        if steps is None:
            self.log("%s sequence is empty: nothing to run" % key)
            self._finish(Mode.RUNNING if mode == Mode.STARTING else Mode.INACTIVE)
            return
        self._abort_event = threading.Event()
        ctx = self._make_ctx(self._abort_event)
        self.runner = SequenceRunner(
            steps, ctx,
            on_done=lambda ok, error: self._sequence_done(key, mode, on_success, ok, error))
        self.runner.start()

    def _make_ctx(self, abort_event):
        return StepContext(self.model, self.client, self.config["devices"], self.log,
                           abort_event=abort_event, roof_timeout_s=self.config["roof_timeout_s"])

    def _sequence_done(self, key, mode, on_success, ok, error):
        self.runner = None
        if ok:
            self._finish(on_success)
            return
        aborted = error == "aborted"
        self.log("%s sequence did not complete: %s" % (key, error or "aborted"))
        self._emit("sequence_failed", {"mode": mode, "error": error or "aborted"})
        if aborted:
            self.mode = Mode.INACTIVE
            self._emit("mode_changed", Mode.INACTIVE)

    def _finish(self, mode):
        self._abort_event = threading.Event()
        if mode == Mode.RUNNING:
            self._start_checks()
        else:
            self._stop_checks()
        self.mode = mode
        self.log("operating mode: %s" % mode)
        self._emit("mode_changed", mode)

    def abort_sequence(self):
        if not self.is_busy():
            return False
        self.log("abort requested: stopping the sequence")
        self._abort_event.set()
        if self.runner is not None:
            self.runner.abort(by_user=True)
        self._stop_checks()
        self.mode = Mode.INACTIVE
        self._emit("mode_changed", Mode.INACTIVE)
        self._emit("sequence_aborted")
        return True

    def send_roof_motion(self, member):
        if not self.roof_move_allowed():
            self._emit("mode_rejected", "roof movement is only allowed in static mode")
            return False
        device = self.config["devices"].get("roof")
        if not device:
            self.log("no roof device configured")
            return False
        try:
            self.client.new_switch(device, "DOME_SHUTTER", [(member, True)])
        except Exception as error:
            self.log("roof command failed: %s" % error)
            return False
        self.log("roof: %s sent" % member)
        return True

    def roof_stop(self):
        if not self.roof_stop_allowed():
            self._emit("mode_rejected", "inactive: change mode first")
            return False
        device = self.config["devices"].get("roof")
        try:
            self.client.new_switch(device, "DOME_ABORT_MOTION", [("ABORT", True)])
        except Exception as error:
            self.log("roof STOP failed: %s" % error)
            return False
        self.log("roof STOP sent")
        return True

    def set_port(self, device_alias, port, on):
        if not self.relay_actions_allowed():
            self._emit("mode_rejected", "relays are only operable in static mode")
            return False
        device = self.config["devices"].get(device_alias, device_alias)
        member = self.model.port_member(device, port)
        if member is None:
            self.log("relay %r not found on %s" % (port, device))
            return False
        try:
            self.client.new_switch(device, "PORTS", [(member, bool(on))])
        except Exception as error:
            self.log("relay command failed: %s" % error)
            return False
        self.log("%s: %s -> %s" % (device, port, "ON" if on else "OFF"))
        return True

    def _start_checks(self):
        try:
            self.checks = CheckRunner(
                self.config["checks"],
                sequence_steps_for=self._steps_for_check,
                ctx_factory=lambda: self._make_ctx(threading.Event()),
                log=self.log,
                monitors=self.monitors)
            self.checks.start()
        except Exception as error:
            self.log("cannot start checks: %s" % error)
            self.checks = None

    def _steps_for_check(self, name):
        for spec in self.config["checks"].get("items", []):
            if spec["name"] == name:
                return spec.get("recovery", [])
        return []

    def _stop_checks(self):
        if self.checks is not None:
            self.checks.stop()
            self.checks = None

    def devices_ready(self):
        missing = []
        aliases = self.config["devices"]
        roof = aliases.get("roof")
        if self.model.get(roof, "DOME_SHUTTER") is None and self.model.get(roof, "DOME_PARK") is None:
            missing.append(roof)
        known = set(self.model.relay_devices())
        for alias in ("relay16", "kmtronic8"):
            device = aliases.get(alias)
            if device is not None and device not in known:
                missing.append(device)
        focuser = aliases.get("focuser")
        if focuser and self.model.get(focuser, "SCOPE_SAFETY") is None:
            missing.append(focuser)
        return missing
