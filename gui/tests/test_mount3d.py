import math

from indi_relay_gui import mount3d


EPS = 1e-6


def approx(v, other):
    assert all(abs(a - b) < EPS for a, b in zip(v, other))


def test_dir_from_azalt_cardinal_directions():
    approx(mount3d.dir_from_azalt(0.0, 0.0), (-1.0, 0.0, 0.0))   # north
    approx(mount3d.dir_from_azalt(90.0, 0.0), (0.0, 1.0, 0.0))   # east
    approx(mount3d.dir_from_azalt(180.0, 0.0), (1.0, 0.0, 0.0))  # south
    approx(mount3d.dir_from_azalt(270.0, 0.0), (0.0, -1.0, 0.0))  # west
    approx(mount3d.dir_from_azalt(0.0, 90.0), (0.0, 0.0, 1.0))   # zenith


def test_celestial_pole_northern_hemisphere():
    az, alt = 0.0, 48.2
    pole = mount3d.celestial_pole(48.2)
    expected = mount3d.dir_from_azalt(az, alt)
    approx(pole, expected)
    assert pole[2] == math.sin(math.radians(48.2))


def test_celestial_pole_southern_hemisphere():
    pole = mount3d.celestial_pole(-33.9)
    approx(pole, mount3d.dir_from_azalt(180.0, 33.9))


def test_dec_axis_perpendicular_and_unit():
    ra_axis = mount3d.dir_from_azalt(0.0, 48.2)
    tube = mount3d.dir_from_azalt(180.0, 0.0)
    dec = mount3d.dec_axis(ra_axis, tube)
    assert abs(mount3d.vec_norm(dec) - 1.0) < EPS
    assert abs(mount3d.vec_dot(dec, ra_axis)) < EPS
    assert abs(mount3d.vec_dot(dec, tube)) < EPS


def test_dec_axis_degenerate_fallback():
    same = mount3d.dir_from_azalt(45.0, 10.0)
    dec = mount3d.dec_axis(same, same)
    assert abs(mount3d.vec_norm(dec) - 1.0) < EPS
    assert abs(mount3d.vec_dot(dec, same)) < EPS   # exactly 90 deg from RA


def test_dec_axis_is_90_deg_from_polar_axis():
    pole = mount3d.celestial_pole(42.5)
    for az, alt in ((180.0, 0.0), (90.0, 45.0), (315.0, 33.0)):
        tube = mount3d.dir_from_azalt(az, alt)
        dec = mount3d.dec_axis(pole, tube)
        assert abs(mount3d.vec_dot(dec, pole)) < EPS
        assert abs(mount3d.vec_dot(dec, tube)) < EPS
        assert abs(mount3d.vec_norm(dec) - 1.0) < EPS


def test_pier_side_flips_dec_axis_saddle_side():
    figure = mount3d.build_figure(
        {"az": 180.0, "alt": 0.0, "pier": "west"}, {"lat": 42.5, "lon": 0.0})
    dec_bar = next(s for s in figure["segments"] if s["color"] == mount3d.DEC_COLOR)
    assert dec_bar["b"][1] < 0.0     # saddle on the west side
    figure = mount3d.build_figure(
        {"az": 180.0, "alt": 0.0, "pier": "east"}, {"lat": 42.5, "lon": 0.0})
    dec_bar = next(s for s in figure["segments"] if s["color"] == mount3d.DEC_COLOR)
    assert dec_bar["b"][1] > 0.0     # saddle on the east side


def test_project_origin_at_center():
    u, v, depth = mount3d.project((0.0, 0.0, 0.0), 200.0, 25.0, 100.0, (50, 40))
    assert (u, v, depth) == (50.0, 40.0, 0.0)


def test_project_depth_grows_with_distance_from_camera():
    # Camera sits at azimuth 200; a point towards az 200 is closer than one
    # at az 20 (opposite side).
    near = mount3d.project(mount3d.dir_from_azalt(200.0, 0.0), 200.0, 25.0, 1.0, (0, 0))
    far = mount3d.project(mount3d.dir_from_azalt(20.0, 0.0), 200.0, 25.0, 1.0, (0, 0))
    assert near[2] < far[2]


def test_tube_direction_prefers_horizontal_coord():
    direction = mount3d._tube_direction(
        {"az": 100.0, "alt": 45.0, "ra": 1.0, "dec": 1.0, "parked": True}, 48.2)
    approx(direction, mount3d.dir_from_azalt(100.0, 45.0))


def test_tube_direction_falls_back_to_equatorial():
    from indi_relay_gui.attitude import alt_az
    az, alt = alt_az(18.5, 38.7939, 48.2, 0.0)
    direction = mount3d._tube_direction({"ra": 18.5, "dec": 38.7939}, 48.2)
    approx(direction, mount3d.dir_from_azalt(az, alt))


def test_tube_direction_parked_reference():
    approx(mount3d._tube_direction({"parked": True}, None),
           mount3d.dir_from_azalt(180.0, 0.0))


def test_tube_direction_unknown():
    assert mount3d._tube_direction({}, None) is None


def test_gem_structure_pillar_ra_bar_dec_bar_tube():
    figure = mount3d.build_figure(
        {"az": 90.0, "alt": 0.0},                           # pointing east
        {"lat": 47.5, "lon": 0.0})
    by_width = {}
    for seg in figure["segments"]:
        by_width.setdefault(seg["width"], []).append(seg)
    pier = by_width[15][0]
    assert pier["a"] == (0.0, 0.0, 0.0)
    assert pier["b"] == (0.0, 0.0, mount3d.PIER_HEIGHT)
    ra_bar = by_width[16][0]
    pole = mount3d.celestial_pole(47.5)
    elevation = math.degrees(math.asin(pole[2]))
    assert abs(elevation - 47.5) < 0.1    # inclined 47.5 from the HORIZONTAL
    midpoint = mount3d.vec_scale(mount3d.vec_add(ra_bar["a"], ra_bar["b"]), 0.5)
    assert midpoint == (0.0, 0.0, mount3d.PIER_HEIGHT)   # centred on the pier top
    assert abs(mount3d.vec_norm(mount3d.vec_sub(ra_bar["b"], ra_bar["a"]))
               - mount3d.RA_LENGTH) < EPS
    assert abs(mount3d.vec_dot(
        mount3d.vec_unit(mount3d.vec_sub(ra_bar["b"], ra_bar["a"])), pole) - 1.0) < EPS
    dec_bar = by_width[11][0]
    dec_dir = mount3d.vec_unit(mount3d.vec_sub(dec_bar["b"], dec_bar["a"]))
    assert abs(mount3d.vec_dot(dec_dir, pole)) < EPS       # 90 deg from polar axis
    assert abs(mount3d.vec_norm(mount3d.vec_sub(dec_bar["b"], dec_bar["a"]))
               - (mount3d.SADDLE_OFFSET + mount3d.DEC_BACK)) < EPS
    # the bar runs from head - DEC_BACK (counterweight side) to the saddle
    head0 = (0.0, 0.0, mount3d.PIER_HEIGHT)
    assert abs(mount3d.vec_norm(mount3d.vec_sub(dec_bar["a"], head0))
               - mount3d.DEC_BACK) < EPS
    saddle_offset_vec = mount3d.vec_sub(dec_bar["b"], head0)
    assert abs(mount3d.vec_dot(saddle_offset_vec, dec_dir)
               - mount3d.SADDLE_OFFSET) < EPS
    tube = [s for s in figure["segments"] if s["color"] == "#2e8b57"]
    assert len(tube) == len(mount3d.TUBE_WIDTHS)
    east = mount3d.dir_from_azalt(90.0, 0.0)
    ordered = sorted(tube, key=lambda s: mount3d.vec_dot(
        mount3d.vec_scale(mount3d.vec_add(s["a"], s["b"]), 0.5), east))
    # telescopic: sections widen towards the lens side
    assert [s["width"] for s in ordered] == list(mount3d.TUBE_WIDTHS)
    front = ordered[-1]
    approx(mount3d.vec_unit(mount3d.vec_sub(front["b"], front["a"])), east)
    points = [p for s in tube for p in (s["a"], s["b"])]
    along = sorted((mount3d.vec_dot(p, east), p) for p in points)
    tail, tip = along[0][1], along[-1][1]
    assert abs(mount3d.vec_norm(mount3d.vec_sub(tip, tail))
               - 2.0 * mount3d.TUBE_HALF) < EPS
    head = (0.0, 0.0, mount3d.PIER_HEIGHT)
    # the tube centreline is attached at the saddle, SADDLE_OFFSET out on the DEC axis
    dec_tip = mount3d.vec_add(head, mount3d.vec_scale(dec_dir, mount3d.SADDLE_OFFSET))
    dec_back = mount3d.vec_sub(head, mount3d.vec_scale(dec_dir, mount3d.DEC_BACK))
    tube_mid = mount3d.vec_scale(mount3d.vec_add(tail, tip), 0.5)
    on_dec_bar = min(mount3d.vec_norm(mount3d.vec_sub(tube_mid, dec_tip)),
                     mount3d.vec_norm(mount3d.vec_sub(tube_mid, dec_back)))
    assert on_dec_bar < EPS


def test_build_figure_parked_has_tube_pointing_south():
    figure = mount3d.build_figure(
        {"parked": True, "ra": None, "dec": None, "az": None, "alt": None,
         "pier": "west"},
        {"lat": 48.2, "lon": 7.6})
    assert len(figure["segments"]) > 2
    assert len(figure["polygons"]) == 1 and len(figure["polygons"][0]["points"]) == mount3d.DISC_POINTS
    assert all(point[2] == 0.0 for point in figure["polygons"][0]["points"])
    south = mount3d.dir_from_azalt(180.0, 0.0)
    tube = [s for s in figure["segments"] if s["color"] == "#2e8b57"]
    ends = [point for s in tube for point in (s["a"], s["b"])]
    tip = max(ends, key=lambda p: mount3d.vec_dot(p, south))
    tail = min(ends, key=lambda p: mount3d.vec_dot(p, south))
    approx(mount3d.vec_unit(mount3d.vec_sub(tip, tail)), south)
    labels = [text for _, text in figure["labels"]]
    assert "lens" in labels and "camera" in labels and "pier" in labels
    kinds = sorted(marker["kind"] for marker in figure["markers"])
    assert kinds == ["camera", "lens"]
    lens = next(m for m in figure["markers"] if m["kind"] == "lens")
    camera = next(m for m in figure["markers"] if m["kind"] == "camera")
    approx(lens["point"], tip)      # lens at the pointing end
    approx(camera["point"], tail)   # camera at the back end


def test_polar_axis_always_drawn():
    figure = mount3d.build_figure({}, {"lat": 48.2, "lon": 7.6})
    assert len(figure["segments"]) == 2      # pier + RA axis (ground is a polygon)
    assert len(figure["polygons"]) == 1 and len(figure["polygons"][0]["points"]) == mount3d.DISC_POINTS
    assert [text for _, text in figure["labels"]] == ["pier", "RA"]
    figure = mount3d.build_figure({}, None)  # no site: RA at the fallback latitude
    assert len(figure["segments"]) == 2      # pier + RA axis + ground polygon
    ra_bar = next(s for s in figure["segments"] if s["color"] == mount3d.RA_COLOR)
    assert abs(mount3d.vec_norm(mount3d.vec_sub(ra_bar["b"], ra_bar["a"]))
               - mount3d.RA_LENGTH) < EPS
    pole = mount3d.celestial_pole(mount3d.FALLBACK_LATITUDE)
    assert abs(mount3d.vec_dot(
        mount3d.vec_unit(mount3d.vec_sub(ra_bar["b"], ra_bar["a"])), pole) - 1.0) < EPS


def test_polar_axis_visible_length_at_45_deg():
    figure = mount3d.build_figure({}, {"lat": 45.0})
    ra_bar = next(s for s in figure["segments"] if s["color"] == mount3d.RA_COLOR)
    assert ra_bar["width"] >= 15
    pole = mount3d.celestial_pole(45.0)      # 45 deg off the vertical, north
    assert abs(mount3d.vec_dot(
        mount3d.vec_unit(mount3d.vec_sub(ra_bar["b"], ra_bar["a"])), pole) - 1.0) < EPS


def test_hour_angle_wraps_to_pm180():
    assert mount3d.hour_angle_deg(18.5, 18.5) == 0.0
    assert abs(mount3d.hour_angle_deg(19.5, 18.5) - 15.0) < EPS
    # LST wrapped past 24h 1h ago: HA = +15 deg (west of the meridian)
    assert abs(mount3d.hour_angle_deg(0.5, 23.5) - 15.0) < EPS


def test_mechanical_axes_meridian_equator_origin_south():
    # HA = 0, DEC = 0: scope points at the meridian celestial equator (south),
    # DEC axis east-west with the saddle on the west side (pier west).
    tube, dec = mount3d.mechanical_axes(
        {"ra": 18.5, "dec": 0.0, "lst": 18.5, "pier": "west"}, 47.5)
    approx(tube, mount3d.dir_from_azalt(180.0, 90.0 - 47.5))
    approx(dec, (0.0, -1.0, 0.0))


def test_mechanical_axes_dec_zero_perpendicular_to_polar_axis():
    pole = mount3d.celestial_pole(47.5)
    tube, dec = mount3d.mechanical_axes(
        {"ra": 10.0, "dec": 0.0, "lst": 10.0, "pier": "west"}, 47.5)
    assert abs(mount3d.vec_dot(tube, pole)) < EPS       # 0 deg = perpendicular
    assert abs(mount3d.vec_dot(dec, pole)) < EPS
    assert abs(mount3d.vec_dot(dec, tube)) < EPS


def test_mechanical_axes_object_west_of_meridian():
    # HA = +2h (30 deg west): the tube leaves the meridian towards the west.
    tube, dec = mount3d.mechanical_axes(
        {"ra": 10.0, "dec": 0.0, "lst": 12.0, "pier": "west"}, 47.5)
    assert tube[1] < 0.0                                # west of the meridian
    assert abs(mount3d.vec_dot(dec, tube)) < EPS
    assert abs(mount3d.vec_dot(dec, mount3d.celestial_pole(47.5))) < EPS


def test_mechanical_axes_dec_rotates_tube_about_dec_axis():
    # DEC = +30: the tube is 30 deg from the equator, 60 deg from the pole.
    pole = mount3d.celestial_pole(47.5)
    tube, _ = mount3d.mechanical_axes(
        {"ra": 10.0, "dec": 30.0, "lst": 10.0, "pier": "west"}, 47.5)
    assert abs(mount3d.vec_dot(tube, pole) - math.sin(math.radians(30.0))) < EPS


def test_mechanical_axes_park_geometry_reproduces_parked_pose():
    # HA = 0, DEC = lat - 90: tube horizontal pointing south.
    lat = 47.5
    tube, dec = mount3d.mechanical_axes(
        {"ra": 5.0, "dec": lat - 90.0, "lst": 5.0, "pier": "west"}, lat)
    approx(tube, mount3d.dir_from_azalt(180.0, 0.0))
    approx(dec, (0.0, -1.0, 0.0))


def test_mechanical_axes_unknown_pier_keeps_base_orientation():
    tube, dec = mount3d.mechanical_axes({"ra": 5.0, "dec": 0.0, "lst": 5.0}, 47.5)
    approx(dec, (0.0, 1.0, 0.0))


def test_mechanical_axes_requires_all_angles():
    assert mount3d.mechanical_axes({"ra": 5.0, "dec": 10.0}, 47.5) is None
    assert mount3d.mechanical_axes({"lst": 5.0, "dec": 10.0}, 47.5) is None
    assert mount3d.mechanical_axes({"ra": 5.0, "lst": 5.0}, 47.5) is None


def test_figure_syncs_to_hour_angle_and_dec():
    mount = {"ra": 10.0, "dec": 25.0, "lst": 11.0, "az": None, "alt": None,
             "pier": "west", "parked": False}
    figure = mount3d.build_figure(mount, {"lat": 47.5, "lon": 0.0})
    tube, dec = mount3d.mechanical_axes(mount, 47.5)
    tube_segs = [s for s in figure["segments"] if s["color"] == mount3d.TUBE_COLOR]
    direction = mount3d.vec_unit(mount3d.vec_sub(
        max(tube_segs, key=lambda s: mount3d.vec_dot(s["b"], tube))["b"],
        min(tube_segs, key=lambda s: mount3d.vec_dot(s["a"], tube))["a"]))
    approx(direction, tube)
    dec_bar = next(s for s in figure["segments"] if s["color"] == mount3d.DEC_COLOR)
    approx(mount3d.vec_unit(mount3d.vec_sub(dec_bar["b"], dec_bar["a"])), dec)


def test_park_overrides_hour_angle():
    # Parked: the figure shows the parked pose even though HA = LST - RA = 60 deg
    figure = mount3d.build_figure(
        {"parked": True, "ra": 5.0, "dec": 10.0, "lst": 9.0, "az": None,
         "alt": None, "pier": "west"}, {"lat": 47.5, "lon": 0.0})
    south = mount3d.dir_from_azalt(180.0, 0.0)
    tube = [s for s in figure["segments"] if s["color"] == mount3d.TUBE_COLOR]
    ends = [point for s in tube for point in (s["a"], s["b"])]
    tip = max(ends, key=lambda p: mount3d.vec_dot(p, south))
    tail = min(ends, key=lambda p: mount3d.vec_dot(p, south))
    approx(mount3d.vec_unit(mount3d.vec_sub(tip, tail)), south)


def test_sort_for_draws_far_first():
    figure = mount3d.build_figure({"az": 90.0, "alt": 0.0}, {"lat": 48.2})
    ordered = mount3d.sort_for_drawing(figure)
    view = mount3d.dir_from_azalt(200.0, 25.0)

    def depth(seg):
        return mount3d.vec_dot(
            mount3d.vec_scale(mount3d.vec_add(seg["a"], seg["b"]), 0.5), view)

    depths = [depth(seg) for seg in ordered]
    assert depths == sorted(depths)
