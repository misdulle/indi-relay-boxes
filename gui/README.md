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

## Pi5 wake-up (direct KMTronic8 HTTP)

The indiserver runs on the pi5, so when the pi5 is down the whole INDI stack -
including the KMTronic8 driver itself - is unreachable. Entering `static`
from `inactive` therefore attempts a wake-up: the GUI pings the pi5
(`ping_host`, default `indiserver.host`) and if it does not answer it switches
the pi5 power relay on the KMTronic8 box **directly over HTTP**, using the
same protocol as the KMTronic8 driver (`common/kmtronic_cmd.h`):

- FF style: `GET /FF0N01` (relay N on) / `GET /FF0N00` (off)
- CGI style (used when the box answers 404 to the FF style): read
  `/status.xml`, then `GET /relays.cgi?relay=N` as a toggle
- basic auth when a user is configured

If `/status.xml` shows the relay is already on, it is power-cycled
(off, ~1 s, on); when the state cannot be read the switch to on is sent
unconditionally. Afterwards the GUI keeps pinging for up to `wait_s`
seconds and, as soon as the pi5 is back, it reconnects to the indiserver and
connects the drivers automatically. All of this runs in the background; the
GUI stays in `static` meanwhile, and every step is written to the log panel
(logs from background threads are marshalled to the GUI thread, so HTTP
errors like 401 or an unreachable box show up there).

```yaml
pi5_wake:
  enabled: true         # off by default
  host: 192.168.1.23    # KMTronic8 box
  port: 80
  user: admin           # empty user = no auth
  password: ""          # stored in plaintext in the yaml!
  relay: 1              # 1..8: the relay powering the pi5
  ping_host: 192.168.1.29   # defaults to indiserver.host
  timeout_s: 3          # HTTP / ping timeout
  wait_s: 90            # how long to watch for the pi5 coming back
```

## Roof buttons and safety

OPEN / CLOSE / STOP are push buttons, not toggles: pressing one sends a single
`DOME_SHUTTER` (or `DOME_ABORT_MOTION`) command which the roof driver uses as a
500 ms pulse on the corresponding relay; the driver handles the pulse-on,
pulse-off timing itself. The GUI sends one command per click and displays the
resulting shutter state reported by the driver.

Because the roof driver pulses its STOP / OPEN / CLOSE relays directly, the
corresponding Relay16 ports (from `eulRoof`'s `ROOF_RELAY_PARAMS`:
`RELAY_STOP`, `RELAY_OPEN`, `RELAY_CLOSE`) are **not** shown as regular relay
switches - the dedicated OPEN / CLOSE / STOP buttons are the only way to
command them.

OPEN and CLOSE are only clickable when the mode conditions are met **and**
the roof can actually move: the scope must be **safe** (the `EulFocuser`
driver reports `SCOPE_SAFETY / SAFE` = Ok, fallback `IMU_SAFE` of the roof
driver, no data counts as *not safe*), the roof motor power must be **on**
(`MOTOR_POWER` light of `ROOF_SAFETY_STATUS`) and the roof driver must be
online. While disabled, hovering a button shows why it is locked ("not in
static mode; roof motor power off", ...). The roof panel's safety list shows
mount parked, IMU, motor power and the driver's move-allowed lights.

## Relay switches

Relays are toggle buttons in two columns: KMTronic8 on the left, Relay16 on
the right. Relays whose name matches `relay_exclude_names` (e.g. "none") and
the roof control relays are hidden. Switching is only possible in `static`
mode.

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
  mount: EQMod Mount   # equatorial mount: RA/DEC, pier side, park

# Calibration of the scope position view (see that section below)
imu_park_quaternion: [0.108337, -0.022705, -0.081787, 0.990479]
imu_optical_axis: +x   # IMU body axis that looks along the tube
scope_view_rotation: 200   # top view rotated 200 deg clockwise: south up,
                           # plus 20 deg for the shed orientation

relay_exclude_names: [none]   # relays with these PORT_NAMES are not shown

pi5_wake:              # direct HTTP switch of the pi5 power relay, see
  enabled: true        #  "Pi5 wake-up" section below
  host: 192.168.1.23
  port: 80
  user: admin
  password: ""         # plaintext in the yaml!
  relay: 1
  ping_host: null      # defaults to indiserver.host
  timeout_s: 3
  wait_s: 90

sequences:             # see "Step vocabulary" below
  starting: [...]
  stopping: [...]

checks:                # monitors feed the Heartbeats panel at all times;
  interval_s: 15       #  running mode evaluates them and reacts with recoveries
  grace_s: 120         # cool-down after a recovery before failures count again
  <name>:              # any name you choose, e.g. pi5_ping / imu_udp
    type: ping         # ping, udp or quaternion
    enabled: true
    host: 192.168.1.30 # ping only
    bind: 0.0.0.0      # udp + quaternion only
    port: 2346         # udp port (IMU broadcasts quaternions on 2346)
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
  `timeout_s`
- **quaternion** - same liveness, but each datagram must decode as an IMU
  quaternion (format `w<w>wa<x>ab<y>bc<z>c`, e.g. the IMU broadcasting on
  UDP 2346); staleness counts from the last *decoded* quaternion and the
  decoded value feeds the scope position view.

Both run as background monitors from GUI start (feeding the Heartbeats
panel); in `running` mode the `CheckRunner` reuses the same monitor
instances (one UDP socket, no double binding) and triggers recovery
sequences when the failure threshold is reached.

## Scope / mount position view

The middle panel visualizes where the telescope points, live, in two
projections: a map-style top view (azimuth, north up / east right) and a
side view (altitude).

- **Measured (green, solid)** - from the IMU quaternion broadcast (UDP,
  `type: quaternion`). The reading is rotated against
  `imu_park_quaternion` (a quaternion captured parked: scope level,
  pointing south, west of the pier), so all drawn angles are relative to
  that pose. `imu_optical_axis` says which IMU body axis looks along the
  tube (default `+x`; flip to `-x`, `+y`... if the board is mounted
  differently).
- **Predicted (blue, dashed)** - from the mount: RA/Dec of the configured
  mount device plus the site coordinates the mount driver reports
  (`GEOGRAPHIC_COORD`), converted to topocentric azimuth/altitude.
- **Tube drawing** - the tube is tangent to the (small) pier circle: the
  tangency side comes from `TELESCOPE_PIER_SIDE` (PIER_WEST / PIER_EAST),
  the pointing azimuth from the IMU (or the parked reference). The filled
  **arrow** marks the front lens (pointing direction), the thick stub at
  the other end marks the camera.
- **View rotation** - `scope_view_rotation` (default 200) rotates the top
  view clockwise by that many degrees: 180 puts south up, +20 matches the
  shed orientation. The N/E/S/W labels stay correct.
- **Parked reference (grey)** - when `TELESCOPE_PARK.PARK` is on and no live
  IMU pointing is available, the tube is drawn in its park pose:
  horizontal, pointing south.
- **Park re-anchoring** - while the mount reports PARK and fresh quaternions
  arrive, the newest reading **is** used as the park reference, so sensor
  yaw drift cannot make the parked scope wander off plain south (the
  configured `imu_park_quaternion` is only the fallback before the first
  parked reading). Once the mount moves, pointings are inferred relative
  to that last parked reading.
- The readout line shows RA/Dec, pier side, park state, the raw
  quaternion with its age, both pointings, the difference angle
  between them and the IMU rotation against the active park reference
  (0 while parked).

If site coordinates are missing or zero, only the IMU pointer is drawn;
without a quaternion check, only the readouts.

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
        attitude.py      quaternion / sky math for the position view
        sequence.py      step engine (used by starting/stopping and recoveries)
        checks.py        ping/udp/quaternion checks + scheduler
        controller.py    operating-mode state machine
        main_window.py   PyQt5 panels (heartbeats, relays, roof, checks, modes, log)
        main.py          entry point
        config.example.yaml
      tests/             pytest suite (python3 -m pytest tests)

## Tests

    cd gui && python3 -m pytest tests
