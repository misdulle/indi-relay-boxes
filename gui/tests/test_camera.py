import os
import sys
import time
import types
from unittest.mock import MagicMock

from indi_relay_gui.camera import RtspReader, strip_url_credentials


def test_strip_url_credentials():
    assert strip_url_credentials(
        "rtsp://user:pass@192.168.1.20:554/Streaming/Channels/2"
    ) == "rtsp://192.168.1.20:554/Streaming/Channels/2"
    assert strip_url_credentials(
        "rtsp://u:p@h:x@host/cam") == "rtsp://host/cam"   # password containing ':'


def test_strip_url_credentials_without_credentials():
    url = "rtsp://192.168.1.20:554/cam"
    assert strip_url_credentials(url) == url


def test_strip_url_credentials_odd_input():
    assert strip_url_credentials(None) is None
    assert strip_url_credentials("user@host") == "user@host"   # no scheme
    assert strip_url_credentials("rtsp://@host/cam") == "rtsp://@host/cam"


class _FakeCapture:
    def __init__(self, frames, fail_open=False):
        self._frames = list(frames)
        self._fail_open = fail_open
        self.released = False

    def isOpened(self):
        return not self._fail_open

    def grab(self):
        return bool(self._frames)

    def retrieve(self):
        return True, self._frames.pop(0)

    def release(self):
        self.released = True


def _install_fake_cv2(monkeypatch, capture):
    fake = types.ModuleType("cv2")
    fake.VideoCapture = MagicMock(return_value=capture)
    monkeypatch.setitem(sys.modules, "cv2", fake)
    return fake


def _wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_reader_delivers_frames(monkeypatch):
    capture = _FakeCapture(["F1", "F2", "F3", "F4"])
    _install_fake_cv2(monkeypatch, capture)
    frames, statuses = [], []
    reader = RtspReader("rtsp://host/cam", fps=8,
                        on_frame=frames.append,
                        on_status=lambda ok, msg: statuses.append((ok, msg)))
    reader.start()
    try:
        assert _wait_until(lambda: len(frames) == 4)
        assert statuses[0] == (True, "streaming")
    finally:
        reader.stop()
        reader.join(timeout=3.0)
    assert not reader.is_alive()
    assert frames == ["F1", "F2", "F3", "F4"]
    assert capture.released


def test_reader_cannot_open_stream(monkeypatch):
    capture = _FakeCapture([], fail_open=True)
    _install_fake_cv2(monkeypatch, capture)
    statuses = []
    reader = RtspReader("rtsp://host/cam",
                        on_status=lambda ok, msg: statuses.append((ok, msg)))
    reader.start()
    try:
        assert _wait_until(lambda: bool(statuses))
    finally:
        reader.stop()
        reader.join(timeout=3.0)
    assert not reader.is_alive()
    assert statuses == [(False, "cannot open the stream (is the camera reachable?)")]
    assert capture.released


def test_reader_without_opencv(monkeypatch):
    monkeypatch.setitem(sys.modules, "cv2", None)   # `import cv2` -> ImportError
    statuses = []
    reader = RtspReader("rtsp://host/cam",
                        on_status=lambda ok, msg: statuses.append((ok, msg)))
    reader.run()                                    # no thread needed
    assert statuses == [(False, "opencv-python is not installed - no camera view")]


def test_reader_uses_tcp_transport(monkeypatch):
    capture = _FakeCapture(["F1"])
    fake = _install_fake_cv2(monkeypatch, capture)
    monkeypatch.delenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", raising=False)
    reader = RtspReader("rtsp://host/cam")
    reader.start()
    try:
        assert _wait_until(lambda: capture.released or fake.VideoCapture.called)
    finally:
        reader.stop()
        reader.join(timeout=3.0)
    assert not reader.is_alive()
    assert fake.VideoCapture.called
    assert os.environ.get(
        "OPENCV_FFMPEG_CAPTURE_OPTIONS") == "rtsp_transport;tcp"
