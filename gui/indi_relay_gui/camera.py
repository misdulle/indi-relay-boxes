"""Low-CPU RTSP view of the shed camera.

A background thread grabs and decodes continuously to stay on the live edge
but only *retrieves* (keeps a frame for display) at the configured fps, so
the footprint is the decode of a low-resolution substream plus a few QImage
conversions per second.  Recommended: the camera's substream (e.g.
rtsp://user:pass@cam/Streaming/Channels/2) at 5-10 fps.
"""

import threading
import time


def strip_url_credentials(url):
    """rtsp://user:pass@host:554/x -> rtsp://host:554/x (for logging)."""
    if not isinstance(url, str) or "@" not in url:
        return url
    scheme, _, rest = url.partition("://")
    if not _:
        return url
    userinfo, _, host = rest.rpartition("@")
    if not userinfo:
        return url
    return "%s://%s" % (scheme, host)


class RtspReader(threading.Thread):
    """Opens an RTSP stream and delivers retrieved frames at `fps`."""

    def __init__(self, url, fps=8, on_frame=None, on_status=None):
        super().__init__(daemon=True, name="rtsp-reader")
        self.url = url
        self.fps = max(1.0, float(fps))
        self.on_frame = on_frame or (lambda frame: None)
        self.on_status = on_status or (lambda ok, message: None)
        self._running = threading.Event()

    def stop(self):
        self._running.set()

    def run(self):
        try:
            import cv2
        except ImportError:
            self.on_status(False, "opencv-python is not installed - no camera view")
            return
        import os
        os.environ.setdefault(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp")
        while not self._running.is_set():
            capture = cv2.VideoCapture(self.url)
            if not capture.isOpened():
                capture.release()
                self.on_status(False, "cannot open the stream (is the camera reachable?)")
                if self._running.wait(5.0):
                    break
                continue
            self.on_status(True, "streaming")
            interval = 1.0 / self.fps
            next_retrieve = 0.0
            while not self._running.is_set():
                if not capture.grab():
                    break
                now = time.monotonic()
                if now < next_retrieve:
                    continue
                next_retrieve = now + interval
                ok, frame = capture.retrieve()
                if ok and frame is not None:
                    self.on_frame(frame)
            capture.release()
            if self._running.is_set():
                break
            self.on_status(False, "stream interrupted - retrying")
            if self._running.wait(5.0):
                break
