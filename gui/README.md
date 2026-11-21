# indi-relay-boxes GUI

PyQt5 control panel for the relay boxes and the roll-off roof driven by the
drivers in this repository (`Relay16`, `KMTronic8`, `eulRoof`, `EulFocuser`).
It speaks the INDI XML protocol directly over TCP - no libindi/python bindings
needed.

## Install

    sudo apt install python3-pyqt5 python3-yaml        # Debian/Ubuntu
    # or: pip install -r requirements.txt

## Run

    python3 -m indi_relay_gui                 # from the gui/ directory
    python3 -m indi_relay_gui --config /path/to/config.yaml

On first start a template config is written to
`~/.config/indi-relay-boxes/config.yaml`; edit it and use "File -> Reload config".

## Operating modes

| Mode        | Behaviour |
|-------------|-----------|
| `inactive`  | Read-only: state and config are displayed. Only mode changes are possible. |
| `static`    | Free manual operation: every relay (all boxes, one button per relay) and roof OPEN/CLOSE. STOP always available. |
| `starting`  | Runs the configured starting sequence (power up relays, open the roof, ...). When every step succeeds the mode switches to `running`. On failure the mode stays `starting` and the error is logged. |
| `running`   | The configured health checks run periodically and self-heal (see below). Relays/roof are locked; STOP stays active; changing to `stopping` ends the session. |
| `stopping`  | Runs the configured stopping sequence (close roof, power down). Then back to `inactive`. |

The big red **STOP** button always sends `DOME_ABORT_MOTION` (the roof driver
answers STOP immediately, whatever its internal state) and is enabled in every
mode except `inactive`. During a sequence, **ABORT sequence** cancels the
remaining steps and returns to `inactive` without switching anything off.

## Device connection

Connecting to the indiserver only lists the running drivers - it does not
switch them on. The GUI therefore sends the full `CONNECTION` switch vector
(`CONNECT` = On, `DISCONNECT` = Off) to every device listed under `devices:`
right after the server link is up, and repeats it every 5 seconds for any
driver that has not confirmed `CONNECT` = On (an unconnected driver publishes
nothing - e.g. `PORTS` is only defined once connected). Each driver's status
appears as a row in the Heartbeats panel; a driver restart or an external
disconnect triggers a new CONNECT automatically.

## Roof buttons and safety

OPEN / CLOSE / STOP are push buttons, not toggles: pressing one sends a single
`DOME_SHUTTER` (or `DOME_ABORT_MOTION`) command which the roof driver uses as a
500 ms pulse on the corresponding relay; the driver handles the pulse-on,
pulse-off timing itself. The GUI sends one command per click and displays the
resulting shutter state reported by the driver.

OPEN and CLOSE are only clickable when the scope is **safe**, i.e. the
`EulFocuser` driver reports `SCOPE_SAFETY / SAFE` = Ok. If the focuser is not
data, the roof driver's `IMU_SAFE` light is used as a fallback; no data at all
counts as *not safe*. The current safety state is shown in the roof panel's
safety list.

## Heartbeats panel

The top panel shows a continuously updating clock plus one liveness row per
watched source: the indiserver link, `SCOPE_SAFETY` freshness (EulFocuser,
refreshed every second by the driver), and every enabled monitor from the
`checks:` section (pi5 ping, IMU UDP, ...). Rows turn green when alive, red
when stale, and grey while waiting for the first data after start.

## Configuration reference

```yaml
indiserver:            # where the drivers run
  host: 192.168.1.29
  port: 7624

devices:               # INDI device names as reported by the drivers
  relay16: Relay16
  kmtronic8: KMTronic8
  roof: eulRoof
  focuser: EulFocuser  # publishes SCOPE_SAFETY; gates roof OPEN/CLOSE

relay_exclude_names: [none]   # relays with these PORT_NAMES are not shown

sequences:             # see "Step vocabulary" below
  starting: [...]
  stopping: [...]

checks:                # monitors feed the Heartbeats panel at all times;
  interval_s: 15       #  running mode evaluates them and reacts with recoveries
  grace_s: 120         # cool-down after a recovery before failures count again
  <name>:              # any name you choose, e.g. pi5_ping / imu_udp
    type: ping         # ping or udp
    enabled: true
    host: 192.168.1.30 # ping only
    bind: 0.0.0.0      # udp only
    port: 9541         # udp port the IMU driver sends to
    timeout_s: 60      # no data for this long = failure (udp)
    failures: 3        # consecutive failures before the recovery runs
    recovery: [...]    # sequence executed once per failure episode

roof_timeout_s: 180    # default budget for roof_open / roof_close steps
```

### Step vocabulary (sequences and recoveries)

| Step | Keys | Effect |
|------|------|--------|
| `relay_on` / `relay_off` | `device` (config alias or INDI name), `port` (relay name as shown, or 1-based number), optional `settle_s` | switch one relay |
| `relay_cycle` | `device`, `port`, `seconds` (off-time, default 10) | power-cycle a relay |
| `wait` | `seconds` | pause |
| `roof_open` / `roof_close` | `timeout` (default `roof_timeout_s`) | send DOME_SHUTTER and wait for the driver's state |
| `roof_stop` | - | STOP pulse (DOME_ABORT_MOTION) |

Relay names are matched against the labels configured in the drivers
(`PORT_NAMES`); `none`-named relays still work in sequences but are hidden
from the panels.

### Built-in checks (running mode)

- **ping** - pings a host (e.g. the Pi5); after `failures` consecutive
  failures the `recovery` sequence runs (typically `relay_cycle` on the
  relay that powers the machine).
- **udp** - listens on a UDP port and fails when no datagram arrives within
  `timeout_s` (e.g. the IMU driver's stream); recovery power-cycles the IMU.

Both run as background monitors from GUI start (feeding the Heartbeats
panel); in `running` mode the `CheckRunner` reuses the same monitor
instances (one UDP socket, no double binding) and triggers recovery
sequences when the failure threshold is reached.

### Adding new checks

Add a class to `indi_relay_gui/checks.py` with `run_check(now) ->
(ok, message)`, `status(now) -> {ok, known, age, message}` and
`start()`/`stop()`, register it in the `FACTORIES` dict, and configure a
block under `checks:`. Recovery actions reuse the full step vocabulary, so
anything doable in a sequence can be a reaction.

## Layout

    gui/
      indi_relay_gui/
        indi.py          INDI client + XML stream parser
        model.py         property tree (relays, roof state, safety lights)
        sequence.py      step engine (used by starting/stopping and recoveries)
        checks.py        ping/udp checks + scheduler
        controller.py    operating-mode state machine
        main_window.py   PyQt5 panels (heartbeats, relays, roof, checks, modes, log)
        main.py          entry point
        config.example.yaml
      tests/             pytest suite (python3 -m pytest tests)

## Tests

    cd gui && python3 -m pytest tests
