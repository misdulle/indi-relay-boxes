# indi-relay-boxes FM

INDI drivers for network relay boxes (shared HTTP client in `common/`).

- `relay16/` : 16-port HTTP relay box (`http://IP/ADDR/..`) -> `indi_relay16`
- `roof/` : roll-off roof (INDI Dome) driven through the relay boxes -> `indi_roof`
- `kmtronic8/` : KMTronic 8-channel WEB LAN relay controller -> `indi_kmtronic8`

## Build (Ubuntu 24.04, INDI 2.x from ppa:mutlaqja/ppa)

    sudo apt install libindi-dev cmake g++ libnova-dev libcfitsio-dev libgsl-dev zlib1g-dev
    mkdir build && cd build && cmake -DCMAKE_INSTALL_PREFIX=/usr .. && make -j4 && sudo make install
    indiserver -v indi_relay16

## indi_relay16

Properties (config is saved with "Save" in the Options tab, ~/.indi/16 Port Relay Box_config.xml):
- `BOX_HOST` IP/host, `BOX_NETWORK` TCP port (80), ADDR (30), HTTP timeout (2000 ms).
  Polling period is INDI's standard POLLING_PERIOD (default 2000 ms).
- `PORT_NAMES` one editable name per port (used as switch labels).
- `PORT_WIRING` per port: ON = normally closed. The `PORTS` switches always show POWER state:
  ON = load powered. For NC ports the relay command and the relay state are inverted.
- `PORTS` 16 switches, `ALL_OFF_CMD` powers every load off (NC-aware, sends commands only to ports that are on).
- Connecting only reads the state; nothing is switched. Disconnecting leaves the relays untouched.
- Protocol: relay n OFF = /ADDR/2(n-1), ON = /ADDR/2(n-1)+1, status = /ADDR/99
  (16 characters, relay 1 first). The ADDR is written as a plain integer ("30").

## indi_kmtronic8

Same properties and behaviour as indi_relay16 (8 ports), plus user/password (HTTP basic auth;
default admin/admin, stored in clear text in the config file). Commands `/FF0N01` / `/FF0N00`,
state from `/status.xml` (`relay1`..`relay8`; the stray `relay0` tag is ignored).
Config file: `~/.indi/KMTronic 8 Relay LAN_config.xml`.

## indi_roof (INDI Dome "Roll Off Roof")

Open = pulse on relay OPEN, close = pulse on relay CLOSE, stop = pulse on relay STOP (16-port box,
default relays 2 / 3 / 1, 500 ms, normally-open wiring; a switch inverts the pulse polarity).
Motor power = one relay of the KMTronic box (default relay 4, ON = powered).

Clients see a Dome with DOME_SHUTTER (open/close), DOME_PARK (park = close, unpark = open) and
DOME_ABORT_MOTION. The limit switches stop the motor; with no feedback the driver assumes the roof
is open/closed when the travel time (ROOF_TRAVEL) has elapsed. After STOP or a safety stop the
state is "unknown". `ROOF_STATE_MARK` sets/remembers the state manually (saved in the config).

Safety (all configurable under SAFETY_POLICY / SAFETY_SOURCES):
- mount parked: snoops `TELESCOPE_PARK` of the mount device (default "EQMod Mount"); parked means
  PARK=On and the vector state Ok. No data from the mount counts as NOT parked.
- scope in IMU safe zone: snoops `SCOPE_SAFETY` of the focuser device (default "RPi Encoder Focuser");
  the focuser refreshes it every second, no update for 5 s counts as NOT safe.
- Open/close are refused unless both are true. STOP is always accepted.
- Motor power is on only while movement is allowed ("Motor power only when safe"). If it becomes
  unsafe while the roof moves: motor power is cut first, then a STOP pulse is sent.
- Disconnecting the driver also switches the motor power off.
- OVERRIDE ignores both checks (power then stays on): for emergencies / broken sensors only.
- All HTTP traffic runs in a worker thread, so an unreachable relay box does not freeze the driver.

All drivers must run in the *same indiserver* (snooping). Config: `~/.indi/Roll Off Roof_config.xml`.

## KMTronic command styles

`COMMAND_STYLE` (kmtronic8) / `POWER_COMMAND_STYLE` (roof): *Auto* tries `/FF0N01` / `/FF0N00` and, if
the box answers 404, switches to `/relays.cgi?relay=N`. That command only toggles, so the driver
reads `/status.xml` first and toggles only when the relay is not already in the wanted state.
Choose the style explicitly and save the configuration to skip the probe.
