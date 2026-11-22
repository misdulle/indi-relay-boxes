"""Direct HTTP control of the KMTronic WEB relay boxes.

Mirrors common/kmtronic_cmd.h so the GUI can switch a relay when the
indiserver is not running (e.g. waking the pi5 that hosts it):

  FF style : GET /FF0N01 (relay N on) / GET /FF0N00 (off)
  CGI style: GET /status.xml then GET /relays.cgi?relay=N (toggle)
  AUTO tries FF first and falls back to CGI when the box answers 404.
"""

import base64
import subprocess
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class KmtronicError(Exception):
    pass


def http_get(host, port, path, user="", password="", timeout_s=3.0):
    """GET http://host:port/path -> (status, body); (0, "") on transport failure."""
    url = "http://%s:%d%s" % (host, int(port), path)
    request = Request(url)
    if user:
        token = base64.b64encode(("%s:%s" % (user, password)).encode()).decode()
        request.add_header("Authorization", "Basic " + token)
    try:
        with urlopen(request, timeout=timeout_s) as response:
            return response.getcode(), response.read().decode("utf-8", "replace")
    except HTTPError as error:
        return error.code, ""
    except (URLError, OSError):
        return 0, ""


def read_relay(host, port, user, password, relay, timeout_s=3.0):
    """Current relay state from /status.xml (relay is 1-based); raises when unsure."""
    status, body = http_get(host, port, "/status.xml", user, password, timeout_s)
    if status != 200:
        raise KmtronicError("HTTP status %s for /status.xml" % (status or "no response"))
    marker = "<relay%d>" % relay
    end = "</relay%d>" % relay
    start = body.find(marker)
    if start < 0:
        raise KmtronicError("/status.xml has no <relay%d>" % relay)
    start += len(marker)
    stop = body.find(end, start)
    if stop < 0:
        raise KmtronicError("/status.xml: unterminated <relay%d>" % relay)
    return body[start:stop].strip() == "1"


def set_relay(host, port, user, password, relay, on, timeout_s=3.0, force_cgi=False):
    """Switch relay (1-based) on/off, FF style with CGI fallback like the driver.

    Returns (path used, True when the relay was already in that state)."""
    if not force_cgi:
        status, _ = http_get(host, port, "/FF%02d%02d" % (relay, 1 if on else 0),
                             user, password, timeout_s)
        if status == 200:
            return "/FF%02d%02d" % (relay, 1 if on else 0), False
        if status != 404:
            raise KmtronicError("HTTP status %s for /FF%02d%02d"
                                % (status or "no response", relay, 1 if on else 0))
    current = read_relay(host, port, user, password, relay, timeout_s)
    if current == on:
        return "/status.xml (already %s)" % ("on" if on else "off"), True
    status, _ = http_get(host, port, "/relays.cgi?relay=%d" % relay,
                         user, password, timeout_s)
    if status != 200:
        raise KmtronicError("HTTP status %s for /relays.cgi?relay=%d"
                            % (status or "no response", relay))
    return "/relays.cgi?relay=%d" % relay, False


def power_on(host, port, user, password, relay, timeout_s=3.0, pause_s=1.0):
    """Make sure the relay is on; power-cycle it first when already on.

    Returns (description, cycled).  When /status.xml cannot be read the
    state is unknown and the FF/CGI switch to on is done unconditionally.
    """
    try:
        already_on = read_relay(host, port, user, password, relay, timeout_s)
    except KmtronicError:
        path, _ = set_relay(host, port, user, password, relay, True, timeout_s)
        return path, False
    if not already_on:
        path, _ = set_relay(host, port, user, password, relay, True, timeout_s)
        return path, False
    off_path, _ = set_relay(host, port, user, password, relay, False, timeout_s)
    time.sleep(pause_s)
    on_path, _ = set_relay(host, port, user, password, relay, True, timeout_s)
    return "%s, then %s" % (off_path, on_path), True


def host_reachable(host, timeout_s=2.0):
    """One ICMP ping; False also when ping cannot run at all."""
    try:
        completed = subprocess.run(
            ["ping", "-c", "1", "-W", str(max(1, int(round(timeout_s)))), host],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            timeout=timeout_s + 1.0)
        return completed.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False
