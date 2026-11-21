from datetime import datetime

import pytest

from indi_relay_gui import attitude


SAMPLE = "w0.108337wa-0.022705ab-0.081787bc0.990479c"


def test_parse_quaternion_sample_datagram():
    assert attitude.parse_quaternion(SAMPLE) == (0.108337, -0.022705, -0.081787, 0.990479)


def test_parse_quaternion_rejects_garbage():
    assert attitude.parse_quaternion(b"imu-data") is None
    assert attitude.parse_quaternion("w0.1a0.2b0.3") is None
    assert attitude.parse_quaternion("") is None


def test_optical_axis_at_park_is_south_level():
    park = attitude.parse_quaternion(SAMPLE)
    az, alt = attitude.optical_axis(park, park)
    assert az == pytest.approx(180.0, abs=1e-6)
    assert alt == pytest.approx(0.0, abs=1e-6)


def test_optical_axis_tracks_yaw():
    park = (1.0, 0.0, 0.0, 0.0)
    az, alt = attitude.optical_axis(attitude.quat_from_axis_angle((0, 0, 1), 90), park)
    assert az == pytest.approx(90.0, abs=1e-6)
    assert alt == pytest.approx(0.0, abs=1e-6)
    az, _ = attitude.optical_axis(attitude.quat_from_axis_angle((0, 0, 1), -45), park)
    assert az == pytest.approx(225.0, abs=1e-6)


def test_optical_axis_tracks_elevation():
    park = (1.0, 0.0, 0.0, 0.0)
    az, alt = attitude.optical_axis(attitude.quat_from_axis_angle((0, 1, 0), 30), park)
    assert az == pytest.approx(180.0, abs=1e-6)
    assert alt == pytest.approx(-30.0, abs=1e-6)


def test_angle_between():
    assert attitude.angle_between((180, 0), (180, 0)) == pytest.approx(0.0)
    assert attitude.angle_between((0, 0), (180, 0)) == pytest.approx(180.0)
    assert attitude.angle_between((180, 0), (180, 30)) == pytest.approx(30.0)


def test_lst_at_j2000_greenwich():
    when = datetime(2000, 1, 1, 12, 0, 0)
    assert attitude.lst_hours(when, 0.0) == pytest.approx(18.6974, abs=0.001)


def test_alt_az_dec_zero_on_meridian():
    when = datetime(2000, 1, 1, 12, 0, 0)
    lst = attitude.lst_hours(when, 0.0)
    az, alt = attitude.alt_az(lst, 0.0, 50.0, 0.0, when)
    assert alt == pytest.approx(40.0, abs=0.01)
    assert az == pytest.approx(180.0, abs=0.01)


def test_alt_az_pole_is_latitude():
    when = datetime(2024, 6, 1, 0, 0, 0)
    az, alt = attitude.alt_az(0.0, 90.0, 45.0, 10.0, when)
    assert alt == pytest.approx(45.0, abs=0.5)
    assert az == pytest.approx(0.0, abs=1.0)


def test_formatting():
    assert attitude.format_ra(18.5) == "18h30m00s"
    assert attitude.format_dec(38.7939) == "+38°47'38\""
    assert attitude.format_dec(-11.5) == "-11°30'00\""
    assert attitude.format_ra(None) == "?"
    assert attitude.format_dec(None) == "?"
