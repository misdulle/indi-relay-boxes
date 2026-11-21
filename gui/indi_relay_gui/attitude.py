"""Position math for the scope / mount visualization (pure stdlib).

The IMU broadcasts quaternions as tag-wrapped decimal numbers, e.g.

    w0.108337wa-0.022705ab-0.081787bc0.990479c

is w=0.108337, x=-0.022705, y=-0.081787, z=0.990479 (each value enclosed by
its own tag letter: w -> w, a -> x, b -> y, c -> z).

All angles in degrees unless a name says hours.  Azimuth is 0..360 clockwise
from north (map convention: north up, east right), altitude -90..90.
"""

import math
import re
from datetime import datetime

_COMPONENT_TAGS = {"w": "w", "a": "x", "b": "y", "c": "z"}
_DATAGRAM_RE = re.compile(r"([wabc])(-?\d+(?:\.\d*)?|-?\.\d+)\1")

_AXIS_VECTORS = {"+x": (1, 0, 0), "-x": (-1, 0, 0), "+y": (0, 1, 0),
                 "-y": (0, -1, 0), "+z": (0, 0, 1), "-z": (0, 0, -1)}

# Parked reference frame: south = +x, east = +y, up = +z (right-handed).
# In park the optical axis is horizontal and points south (see config
# imu_optical_axis / imu_park_quaternion).


def parse_quaternion(text):
    """Return (w, x, y, z) or None when the datagram is not a quaternion."""
    if isinstance(text, (bytes, bytearray)):
        text = bytes(text).decode("utf-8", "replace")
    found = {}
    for tag, value in _DATAGRAM_RE.findall(text):
        found.setdefault(_COMPONENT_TAGS[tag], float(value))
    if set(found) != {"w", "x", "y", "z"}:
        return None
    return (found["w"], found["x"], found["y"], found["z"])


def quat_normalize(q):
    w, x, y, z = q
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / norm, x / norm, y / norm, z / norm)


def quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def quat_multiply(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_from_axis_angle(axis, degrees):
    ax, ay, az = axis
    norm = math.sqrt(ax * ax + ay * ay + az * az)
    if norm == 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    ax, ay, az = ax / norm, ay / norm, az / norm
    half = math.radians(degrees) / 2.0
    s = math.sin(half)
    return (math.cos(half), ax * s, ay * s, az * s)


def quat_rotate(q, vector):
    w, x, y, z = quat_normalize(q)
    vx, vy, vz = vector
    # t = 2 q_vec cross v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def optical_axis(quaternion, park_quaternion, axis="+x"):
    """Azimuth/altitude of the scope axis relative to the park pose.

    q_rel = conj(park) * current; at park this is identity, i.e. the axis
    points south, horizontally.  Work only in mount-park / IMU-void terms:
    the quaternion delivered must have been captured with the parked mount.
    """
    if axis not in _AXIS_VECTORS:
        raise ValueError("unknown optical axis %r" % axis)
    rel = quat_multiply(quat_conjugate(park_quaternion), quat_normalize(quaternion))
    x, y, z = quat_rotate(rel, _AXIS_VECTORS[axis])
    az = math.degrees(math.atan2(y, -x)) % 360.0
    alt = math.degrees(math.asin(max(-1.0, min(1.0, z))))
    return az, alt


def angle_between(first, second):
    """Angle between two (azimuth, altitude) sky directions, degrees."""
    az1, alt1 = map(math.radians, first)
    az2, alt2 = map(math.radians, second)
    cos_angle = (math.sin(alt1) * math.sin(alt2)
                 + math.cos(alt1) * math.cos(alt2) * math.cos(az1 - az2))
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_angle))))


def _julian_day(when):
    """Julian date from a UTC datetime (POSIX-epoch based)."""
    import calendar
    timestamp = calendar.timegm(when.utctimetuple()) + when.microsecond / 1e6
    return timestamp / 86400.0 + 2440587.5


def lst_hours(when, lon_deg):
    """Local sidereal time in hours ( Accounts for eastern longitudes +)."""
    d = _julian_day(when) - 2451545.0
    gmst_deg = (280.46061837 + 360.98564736629 * d) % 360.0
    return ((gmst_deg + lon_deg) / 15.0) % 24.0


def hour_angle(ra_hours, lon_deg, when=None):
    """-12..12 h, negative east of the meridian."""
    when = when or datetime.utcnow()
    ha = (lst_hours(when, lon_deg) - ra_hours + 12.0) % 24.0 - 12.0
    return ha


def alt_az(ra_hours, dec_deg, lat_deg, lon_deg, when=None):
    """Where the mount is pointing (topocentric), from RA/Dec and site."""
    when = when or datetime.utcnow()
    ha = math.radians(hour_angle(ra_hours, lon_deg, when) * 15.0)
    dec = math.radians(dec_deg)
    lat = math.radians(lat_deg)
    sin_alt = (math.sin(dec) * math.sin(lat)
               + math.cos(dec) * math.cos(lat) * math.cos(ha))
    alt = math.asin(max(-1.0, min(1.0, sin_alt)))
    cos_az = (math.sin(dec) - math.sin(alt) * math.sin(lat))
    cos_az /= max(1e-9, math.cos(alt) * math.cos(lat))
    az = math.acos(max(-1.0, min(1.0, cos_az)))
    if math.sin(ha) > 0.0:
        az = 2.0 * math.pi - az
    return math.degrees(az) % 360.0, math.degrees(alt)


def format_ra(ra_hours):
    if ra_hours is None:
        return "?"
    total = round(ra_hours * 3600)
    return "%dh%02dm%02ds" % (total // 3600, (total // 60) % 60, total % 60)


def format_dec(dec_deg):
    if dec_deg is None:
        return "?"
    sign = "-" if dec_deg < 0 else "+"
    total = round(abs(dec_deg) * 3600)
    return "%s%d\N{DEGREE SIGN}%02d'%02d\"" % (sign, total // 3600, (total // 60) % 60, total % 60)
