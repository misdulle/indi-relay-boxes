# indi-eul-focuser

Closed-loop stepper focuser for a Raspberry Pi 5: STEP/DIR/ENABLE on GPIO, position
feedback from a UDP stream (Arduino + mechanical encoder). All units are micrometres.

## Build (Ubuntu 24.04 / Raspberry Pi 5)

    sudo apt-add-repository ppa:mutlaqja/ppa     # INDI 2.x (Noble's own libindi-dev is 1.9.9)
    sudo apt update
    sudo apt install libindi-dev cmake g++ libnova-dev libcfitsio-dev libgsl-dev zlib1g-dev
    mkdir build && cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr ..
    make -j4 && sudo make install

No libgpiod is needed: the driver talks to /dev/gpiochipN through the kernel uAPI.

If compilation fails around `FocusAbsPosNP`, your INDI is on the other side of the
legacy/wrapper API change: build with `-DEF_FOCUS_WRAPPED=1` or `=0`
(`cmake -DCMAKE_CXX_FLAGS=-DEF_FOCUS_WRAPPED=1 ..`).

## Run without root

    sudo usermod -aG gpio,dialout $USER           # /dev/gpiochip* is group "gpio" on Ubuntu for Pi
    ls -l /dev/gpiochip4                          # check group; add a udev rule if it is root only:
    # /etc/udev/rules.d/99-gpio.rules:  SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
    # real-time step thread (optional, recommended): /etc/security/limits.d/99-rt.conf
    #   @gpio  -  rtprio  60
    #   @gpio  -  memlock unlimited
    # log out/in, then:
    indiserver -v indi_eul_focuser

Check which gpiochip carries the header pins with `gpioinfo` (the Pi 5 header is normally
gpiochip4, named pinctrl-rp1). STEP=23, DIR=24, ENABLE=25 are BCM numbers = line offsets there.

## Arduino side

UDP datagrams to `<pi>:<UDP port>` (default 2345). One integer per line, in encoder counts
(default 1 count = 1 um, setting `ENCODER_UM_PER_COUNT`; the stream is already in um). Lines starting with `ypr` are ignored.
Send on change, and at least once per second.

## Position persistence

Position = counts * um_per_count + offset. INDI's own config file
(`~/.indi/RPi Encoder Focuser_config.xml`) stores the offset and last position:
- *Sync* sets the offset and saves it.
- "Position at connect" = *Use saved offset*: for an Arduino that keeps its absolute count.
- "Position at connect" = *Restore last position*: for an Arduino whose count restarts at 0 on
  power-up; the offset is rebuilt from the last saved position on the first reading.
Use "Save Configuration" in the Options tab to keep all tuning values.

## Motion behaviour

The encoder is read before every step pulse. Step period is `DELAY_STEP_US` (125 us), slows to
`DELAY_FINE_US` inside the fine zone, optional trapezoid ramp. Stops inside tolerance; if it
overshoots it reverses (one "pass"); more than MAX_PASSES -> alert. Alerts also on stall
(steps without encoder change, allowance = stall margin + backlash after a reversal), loss of UDP
data during a move, travel limits, GPIO errors. Motor stays enabled for `HOLD_MS` after a move.
Settings changed during a move apply to the next move.
