"""Sequences of relay / roof actions, shared by the operating modes and the checks.

A step is a dict loaded from YAML with a "step" key:
  relay_on / relay_off / relay_cycle : device (alias or INDI name), port (name or 1-based index),
                                       seconds (relay_cycle: off-time, default 10)
  wait                               : seconds
  roof_open / roof_close             : timeout (seconds, overrides roof_timeout_s)
  roof_stop : immediate DOME_ABORT_MOTION

Steps run strictly in order; any failure aborts the sequence.
"""

import threading
import time

ABORT_POLL_S = 0.25


class StepError(Exception):
    pass


class StepContext:
    """Everything a step needs: model access, command sending, timing, logging."""

    def __init__(self, model, client, devices, log, abort_event=None,
                 roof_timeout_s=180, clock=time.time, sleeper=None):
        self.model = model
        self.client = client
        self.devices = devices
        self.log = log
        if abort_event is None:
            abort_event = threading.Event()
        self.abort_event = abort_event
        self.roof_timeout_s = roof_timeout_s
        self.clock = clock
        self._sleeper = sleeper if sleeper is not None else self._make_sleeper()
        self.roof_state = self._default_roof_state

    def _make_sleeper(self):
        abort_event = self.abort_event

        def _sleep(seconds):
            abort_event.wait(seconds)

        return _sleep

    def _default_roof_state(self, _device):
        state = self.model.roof_state(_device)
        if isinstance(state, dict):
            return state.get("state") or "unknown"
        return state or "unknown"

    def is_aborted(self):
        return self.abort_event.is_set()

    def abort(self):
        self.abort_event.set()

    def sleep(self, seconds):
        if self.is_aborted():
            raise StepError("aborted")
        self._sleeper(seconds)

    def sleep_until(self, deadline, predicate):
        while not predicate():
            if self.is_aborted():
                raise StepError("aborted")
            if self.clock() >= deadline:
                return False
            self._sleeper(ABORT_POLL_S)
        return True

    def resolve_device(self, alias):
        if not alias:
            raise StepError("step is missing a 'device'")
        if alias in self.devices:
            return self.devices[alias]
        for value in self.devices.values():
            if value == alias:
                return alias
        raise StepError("unknown device %r (not in devices config)" % alias)

    def set_port(self, device_alias, port, on):
        device = self.resolve_device(device_alias)
        member = self.model.port_member(device, port)
        if member is None:
            raise StepError("relay %r not found on device %r" % (port, device))
        self.client.new_switch(device, "PORTS", [(member, bool(on))])
        time.sleep(0.2)
        self.log("%s: %s -> %s" % (device, port, "ON" if on else "OFF"))

    def run_step(self, step):
        kind = step["step"]
        seconds = lambda key="seconds", default=1.0: float(step.get(key, default))
        if kind in ("relay_on", "relay_off"):
            self.set_port(step.get("device"), step.get("port"), kind == "relay_on")
            settle = step.get("settle_s")
            if settle:
                self.sleep(float(settle))
        elif kind == "relay_cycle":
            self.set_port(step.get("device"), step.get("port"), False)
            self.sleep(seconds(default=10.0))
            self.set_port(step.get("device"), step.get("port"), True)
        elif kind == "wait":
            self.sleep(seconds())
        elif kind in ("roof_open", "roof_close"):
            self.roof_move(kind == "roof_open", step)
        elif kind == "roof_stop":
            self.roof_stop_pulse()
        else:
            raise StepError("unknown step %r" % kind)

    def roof_move(self, opening, step):
        device = self.devices.get("roof")
        if not device:
            raise StepError("no roof device configured")
        state = self.model.roof_state(device).get("state")
        wanted = "open" if opening else "closed"
        if state == wanted:
            self.log("roof already %s" % wanted)
            return
        timeout = float(step.get("timeout", self.roof_timeout_s))
        self.client.new_switch(device, "DOME_SHUTTER",
                               [("SHUTTER_OPEN" if opening else "SHUTTER_CLOSE", True)])
        self.log("roof %s commanded (%.0f s budget)" % ("open" if opening else "close", timeout))
        deadline = self.clock() + timeout
        success = self.sleep_until(deadline, lambda: self.roof_state_check(device) in (wanted, "error"))
        state = self.roof_state_check(device)
        if state == "error":
            raise StepError("roof reports an error while moving")
        if not success:
            raise StepError("roof did not reach %s after %.0f s" % (wanted, timeout))
        self.log("roof is %s" % wanted)

    def roof_state_check(self, device):
        value = self.roof_state(device)
        if isinstance(value, dict):
            value = value.get("state")
        return (value or "unknown").lower()

    def roof_stop_pulse(self):
        device = self.devices.get("roof")
        if not device:
            raise StepError("no roof device configured")
        self.client.new_switch(device, "DOME_ABORT_MOTION", [("ABORT", True)])
        self.log("roof STOP pulse sent")


def execute(steps, ctx, on_start=None, on_done=None):
    for index, step in enumerate(steps, start=1):
        if ctx.is_aborted():
            raise StepError("aborted")
        if on_start:
            on_start(index, len(steps), step)
        try:
            ctx.run_step(step)
        except StepError:
            raise
        except Exception as error:
            raise StepError("%s: %s" % (step.get("step"), error)) from error
    if on_done:
        on_done(True)


class SequenceRunner(threading.Thread):
    def __init__(self, steps, ctx, on_done, on_step=None):
        super().__init__(daemon=True, name="sequence-runner")
        self.steps = steps
        self.ctx = ctx
        self.on_done = on_done
        self.on_step = on_step
        self._error = None
        self._aborted_by_user = False

    def result(self):
        return self._error

    def aborted_by_user(self):
        return self._aborted_by_user

    def abort(self, by_user=True):
        self._aborted_by_user = by_user
        self.ctx.abort()

    def run(self):
        self.ctx.log("sequence: %d step(s)" % len(self.steps))
        try:
            execute(self.steps, self.ctx,
                    on_start=self._announce,
                    on_done=lambda ok: self.ctx.log("sequence: all steps done"))
        except StepError as error:
            self._error = str(error)
            self.ctx.log("sequence FAILED: %s" % error)
            self.on_done(False, str(error))
            return
        if self.ctx.is_aborted():
            self.on_done(False, "aborted")
            return
        self.on_done(True, None)

    def _announce(self, index, total, step):
        self.ctx.log("sequence step %d/%d: %s" % (index, total, step.get("step")))
        if self.on_step:
            self.on_step(index, total, step)
