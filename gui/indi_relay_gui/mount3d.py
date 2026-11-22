"""Stick-figure 3D representation of a german equatorial mount.

Pure python (no numpy): small vector helpers, a fixed orthographic camera
and the figure builder.  World frame: +x = south, +y = east, +z = up
(same azimuth convention as attitude.py: az = atan2(y, -x)).
"""

import math

PIER_HEIGHT = 1.0     # m, pier top above ground
RA_LENGTH = 0.35      # m, RA housing along the pole axis
DEC_LENGTH = 0.30     # m, DEC housing / saddle offset from the RA axis
TUBE_HALF = 0.55      # m, half length of the drawn tube


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
    """DEC axis: perpendicular to both the RA axis and the tube."""
    axis = vec_cross(ra_axis, tube_dir)
    if vec_norm(axis) < 1e-6:
        return (0.0, 1.0, 0.0)
    return vec_unit(axis)


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


def build_figure(mount, site=None, view_az=200.0, view_alt=25.0):
    """Stick figure of the mount from its reported angles.

    mount: dict like model.mount_state (ra, dec, az, alt, pier, parked).
    site: {"lat": .., "lon": ..} or None.  Returns a dict with "segments",
    "labels", "view" keys; empty when nothing is known.
    """
    lat = None
    if site and site.get("lat"):
        lat = site["lat"]
    figure = {"segments": [], "labels": [], "view": (view_az, view_alt)}
    segments = figure["segments"]
    labels = figure["labels"]

    origin = (0.0, 0.0, 0.0)
    head = (0.0, 0.0, PIER_HEIGHT)
    segments.append(_seg(origin, head, "#555555", 9))
    labels.append((vec_scale(head, 1.02), "pier"))

    ra_dir = celestial_pole(lat) if lat is not None else None
    if ra_dir is not None:
        ra_tip = vec_add(head, vec_scale(ra_dir, RA_LENGTH))
        segments.append(_seg(head, ra_tip, "#8e44ad", 11))
        labels.append((vec_add(ra_tip, vec_scale(ra_dir, 0.08)), "RA"))

    tube_dir = _tube_direction(mount, lat)
    if tube_dir is None:
        return figure

    ra_axis = ra_dir or (0.0, -1.0, 0.0)
    dec = dec_axis(ra_axis, tube_dir)
    saddle = vec_add(head, vec_scale(dec, DEC_LENGTH))
    segments.append(_seg(head, saddle, "#8e44ad", 8))
    tail = vec_add(saddle, vec_scale(tube_dir, -TUBE_HALF))
    tip = vec_add(saddle, vec_scale(tube_dir, TUBE_HALF))
    segments.append(_seg(tail, tip, "#2e8b57", 12))
    labels.append((vec_add(tip, vec_scale(tube_dir, 0.08)), "lens"))
    labels.append((vec_add(tail, vec_scale(tube_dir, -0.08)), "camera"))
    return figure


def _tube_direction(mount, lat):
    """Best available pointing of the optical axis (unit vector or None)."""
    if mount.get("az") is not None and mount.get("alt") is not None:
        return dir_from_azalt(mount["az"], mount["alt"])
    if (mount.get("ra") is not None and mount.get("dec") is not None
            and lat is not None):
        from .attitude import alt_az
        az, alt = alt_az(mount["ra"], mount["dec"], lat, 0.0)
        return dir_from_azalt(az, alt)
    if mount.get("parked"):
        return dir_from_azalt(180.0, 0.0)
    return None


def sort_for_drawing(figure):
    """Far segments first so nearer sticks paint on top."""
    view_az, view_alt = figure["view"]
    view_dir = dir_from_azalt(view_az, view_alt)

    def depth(seg):
        return vec_dot(vec_scale(vec_add(seg["a"], seg["b"]), 0.5), view_dir)

    return sorted(figure["segments"], key=depth)
