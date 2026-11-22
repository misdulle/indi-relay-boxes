"""Stick-figure 3D representation of a german equatorial mount.

Pure python (no numpy): small vector helpers, a fixed orthographic camera
and the figure builder.  World frame: +x = south, +y = east, +z = up
(same azimuth convention as attitude.py: az = atan2(y, -x)).
"""

import math

PIER_COLOR = "#a0522d"   # sienna: clearly distinct from the axes
RA_COLOR = "#1565c0"     # polar (hour angle) axis
DEC_COLOR = "#d32f2f"    # declination axis
TUBE_COLOR = "#2e8b57"

PIER_HEIGHT = 0.80    # m, vertical pier
RA_LENGTH = 0.20      # m, polar (hour angle) bar, pointing at the NCP
FALLBACK_LATITUDE = 45.0   # used when the mount reports no site latitude
DEC_BACK = 0.15       # m, DEC bar extends this far on the counterweight side
SADDLE_OFFSET = 0.20  # m, tube attachment offset along the DEC axis
TUBE_HALF = 0.55      # m, half length of the drawn tube
TUBE_WIDTHS = (13, 16, 21)   # telescopic look: wider towards the lens
GROUND_RADIUS = 0.60  # m, ground disc at the base of the pier
DISC_POINTS = 36


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(a, factor):
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def vec_dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vec_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def vec_norm(a):
    return math.sqrt(vec_dot(a, a))


def vec_unit(a):
    n = vec_norm(a)
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return (a[0] / n, a[1] / n, a[2] / n)


def dir_from_azalt(az_deg, alt_deg):
    """Unit direction for an azimuth/altitude in the sky-direction sense."""
    az = math.radians(az_deg)
    alt = math.radians(alt_deg)
    return (-math.cos(alt) * math.cos(az), math.cos(alt) * math.sin(az),
            math.sin(alt))


def celestial_pole(lat_deg):
    """Direction of the celestial pole as seen from a latitude."""
    return dir_from_azalt(0.0 if lat_deg >= 0 else 180.0, abs(lat_deg))


def dec_axis(ra_axis, tube_dir):
    """DEC axis: exactly 90 deg from the polar axis (and from the tube)."""
    axis = vec_cross(ra_axis, tube_dir)
    if vec_norm(axis) < 1e-6:
        axis = vec_cross(ra_axis, (0.0, 0.0, 1.0) if abs(ra_axis[2]) < 0.9
                         else (0.0, 1.0, 0.0))
    dec = vec_unit(axis)
    return vec_unit(vec_sub(dec, vec_scale(ra_axis, vec_dot(dec, ra_axis))))


def _pier_side_adjust(dec, pier):
    """Flip the DEC axis so the saddle sits on the actual scope side."""
    if pier == "west" and dec[1] > 1e-9:
        return vec_scale(dec, -1.0)
    if pier == "east" and dec[1] < -1e-9:
        return vec_scale(dec, -1.0)
    return dec


def hour_angle_deg(lst_hours, ra_hours):
    """Instrument hour angle HA = LST - RA, wrapped to (-180, 180] deg."""
    return ((lst_hours - ra_hours) * 15.0 + 180.0) % 360.0 - 180.0


def mechanical_axes(mount, lat):
    """Tube + DEC axis from the reported mechanical angles, or None.

    HA = TIME_LST.LST - EQUATORIAL_EOD_COORD.RA rotates the DEC assembly
    about the polar axis (origin: scope pointing south on the meridian);
    DEC rotates the tube about the DEC axis (0 deg = perpendicular to the
    polar axis, i.e. pointing at the celestial equator).
    """
    ra, dec_deg, lst = mount.get("ra"), mount.get("dec"), mount.get("lst")
    if ra is None or dec_deg is None or lst is None:
        return None
    lat_deg = lat if lat is not None else FALLBACK_LATITUDE
    phi = math.radians(lat_deg)
    pole = celestial_pole(lat_deg)
    south = (math.sin(phi), 0.0, math.cos(phi))   # meridian celestial equator
    east = (0.0, 1.0, 0.0)
    h = math.radians(hour_angle_deg(lst, ra))
    d = math.radians(dec_deg)
    equator = vec_sub(vec_scale(south, math.cos(h)), vec_scale(east, math.sin(h)))
    tube = vec_unit(vec_add(vec_scale(pole, math.sin(d)),
                            vec_scale(equator, math.cos(d))))
    dec = vec_unit(vec_add(vec_scale(east, math.cos(h)),
                           vec_scale(south, math.sin(h))))
    dec = _pier_side_adjust(dec, mount.get("pier"))
    return tube, dec


def project(point, view_az_deg, view_alt_deg, scale, center):
    """Orthographic projection; returns (x, y, depth) in screen points.

    The camera looks at the origin from (view_az_deg, view_alt_deg).
    """
    view_dir = dir_from_azalt(view_az_deg, view_alt_deg)
    forward = vec_scale(view_dir, -1.0)
    right = vec_unit(vec_cross((0.0, 0.0, 1.0), forward))
    up = vec_cross(forward, right)
    u = vec_dot(point, right) * scale + center[0]
    v = -vec_dot(point, up) * scale + center[1]
    depth = -vec_dot(point, view_dir)
    return (u, v, depth)


def _seg(a, b, color, width):
    return {"a": a, "b": b, "color": color, "width": width}


def _ground_disc(radius=GROUND_RADIUS, points=DISC_POINTS):
    """Horizontal filled disc at the pier base, materializing the ground."""
    ring = [(radius * math.cos(2.0 * math.pi * i / points),
             radius * math.sin(2.0 * math.pi * i / points), 0.0)
            for i in range(points)]
    return {"points": ring, "color": "#77848c", "alpha": 70,
            "edge": "#3f484d"}


def build_figure(mount, site=None, view_az=200.0, view_alt=25.0):
    """Stick figure of the mount from its reported angles.

    mount: dict like model.mount_state (ra, dec, lst, az, alt, pier,
    parked).  site: {"lat": .., "lon": ..} or None.  Returns a dict with
    "segments", "labels", "view" keys; empty when nothing is known.
    """
    lat = None
    if site and site.get("lat"):
        lat = site["lat"]
    figure = {"segments": [], "labels": [], "markers": [], "polygons": [],
              "view": (view_az, view_alt)}
    segments = figure["segments"]
    labels = figure["labels"]
    markers = figure["markers"]

    origin = (0.0, 0.0, 0.0)
    head = (0.0, 0.0, PIER_HEIGHT)
    figure["polygons"].append(_ground_disc())
    segments.append(_seg(origin, head, PIER_COLOR, 15))
    labels.append((vec_scale(head, 1.02), "pier"))

    ra_dir = celestial_pole(lat if lat is not None else FALLBACK_LATITUDE)
    back = vec_sub(head, vec_scale(ra_dir, RA_LENGTH / 2.0))
    tip = vec_add(head, vec_scale(ra_dir, RA_LENGTH / 2.0))
    segments.append(_seg(back, tip, RA_COLOR, 16))
    labels.append((vec_add(tip, vec_scale(ra_dir, 0.10)), "RA"))

    ra_axis = ra_dir
    axes = None
    if not mount.get("parked"):
        axes = mechanical_axes(mount, lat)
    if axes is not None:
        tube_dir, dec = axes
    else:
        tube_dir = _tube_direction(mount, lat)
        if tube_dir is None:
            return figure
        dec = _pier_side_adjust(dec_axis(ra_axis, tube_dir), mount.get("pier"))
    dec_start = vec_sub(head, vec_scale(dec, DEC_BACK))
    saddle = vec_add(head, vec_scale(dec, SADDLE_OFFSET))
    segments.append(_seg(dec_start, saddle, DEC_COLOR, 11))
    labels.append((vec_add(saddle, vec_scale(dec, 0.08)), "DEC"))
    tail = vec_add(saddle, vec_scale(tube_dir, -TUBE_HALF))
    tip = vec_add(saddle, vec_scale(tube_dir, TUBE_HALF))
    step = 2.0 * TUBE_HALF / len(TUBE_WIDTHS)
    for index, width in enumerate(TUBE_WIDTHS):
        start = vec_add(saddle, vec_scale(tube_dir, -TUBE_HALF + index * step))
        end = vec_add(saddle, vec_scale(tube_dir, -TUBE_HALF + (index + 1) * step))
        segments.append(_seg(start, end, TUBE_COLOR, width))
    markers.append({"kind": "lens", "point": tip, "color": TUBE_COLOR})
    markers.append({"kind": "camera", "point": tail, "color": "#4a4a4a"})
    labels.append((vec_add(tip, vec_scale(tube_dir, 0.12)), "lens"))
    labels.append((vec_add(tail, vec_scale(tube_dir, -0.12)), "camera"))
    return figure


def _tube_direction(mount, lat):
    """Best available pointing of the optical axis (unit vector or None)."""
    if mount.get("az") is not None and mount.get("alt") is not None:
        return dir_from_azalt(mount["az"], mount["alt"])
    if mount.get("parked"):
        return dir_from_azalt(180.0, 0.0)
    if (mount.get("ra") is not None and mount.get("dec") is not None
            and lat is not None):
        from .attitude import alt_az
        az, alt = alt_az(mount["ra"], mount["dec"], lat, 0.0)
        return dir_from_azalt(az, alt)
    return None


def sort_for_drawing(figure):
    """Far segments first so nearer sticks paint on top."""
    view_az, view_alt = figure["view"]
    view_dir = dir_from_azalt(view_az, view_alt)

    def depth(seg):
        return vec_dot(vec_scale(vec_add(seg["a"], seg["b"]), 0.5), view_dir)

    return sorted(figure["segments"], key=depth)
