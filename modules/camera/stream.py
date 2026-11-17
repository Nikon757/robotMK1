"""OAK-D Lite colour camera as a live MJPEG stream you can open in any browser.

The JPEG encoding runs on the camera's own video encoder, so streaming costs the Pi almost no
CPU. Uses the DepthAI v2 API (depthai>=2.30,<3).

    from modules.camera import MjpegServer, OakMjpegCamera
    with OakMjpegCamera(fps=15) as camera, MjpegServer(camera, port=8080) as server:
        print(server.urls())
        ...                     # the stream runs in the background
"""
from __future__ import annotations

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

PAGE = b"""<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>RobotMK1 camera</title>
<style>body{margin:0;background:#111;color:#bbb;font-family:sans-serif;text-align:center}
img{max-width:100%;height:auto;display:block;margin:0 auto}</style></head>
<body><img src="/stream" alt="camera stream"><p>RobotMK1 &middot; OAK-D Lite</p></body></html>
"""


class OakMjpegCamera:
    """Colour camera at 640x360, JPEG-encoded on the OAK-D Lite."""

    def __init__(self, fps: int = 15, quality: int = 80):
        self.fps = fps
        self.quality = quality
        self._frame: Optional[bytes] = None
        self._frame_id = 0
        self._new_frame = threading.Condition()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._device = None

    @property
    def running(self) -> bool:
        return self._thread is not None and not self._stop.is_set()

    def start(self) -> "OakMjpegCamera":
        import depthai as dai           # imported here, so the rest of the package works without it

        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setIspScale(1, 3)           # 1920x1080 -> 640x360
        cam.setVideoSize(640, 360)
        cam.setFps(self.fps)
        encoder = pipeline.create(dai.node.VideoEncoder)
        encoder.setDefaultProfilePreset(self.fps, dai.VideoEncoderProperties.Profile.MJPEG)
        encoder.setQuality(self.quality)
        cam.video.link(encoder.input)
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName("mjpeg")
        encoder.bitstream.link(out.input)

        self._device = dai.Device(pipeline)
        queue = self._device.getOutputQueue("mjpeg", maxSize=4, blocking=False)
        self._thread = threading.Thread(target=self._read, args=(queue,), name="oak-mjpeg", daemon=True)
        self._thread.start()
        return self

    def _read(self, queue) -> None:
        while not self._stop.is_set():
            packet = queue.tryGet()
            if packet is None:
                time.sleep(0.005)
                continue
            jpeg = packet.getData().tobytes()
            with self._new_frame:
                self._frame = jpeg
                self._frame_id += 1
                self._new_frame.notify_all()

    def wait_frame(self, after_id: int = 0, timeout: float = 2.0) -> tuple[int, Optional[bytes]]:
        """The newest frame newer than after_id, as (frame id, JPEG bytes).
        Returns (after_id, None) if none arrives within timeout."""
        with self._new_frame:
            self._new_frame.wait_for(lambda: self._frame_id > after_id or self._stop.is_set(), timeout)
            if self._frame_id > after_id:
                return self._frame_id, self._frame
            return after_id, None

    def stop(self) -> None:
        self._stop.set()
        with self._new_frame:
            self._new_frame.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._device is not None:
            self._device.close()
            self._device = None

    def __enter__(self) -> "OakMjpegCamera":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()


class MjpegServer:
    """Serves a camera over HTTP: / (page), /stream (MJPEG), /frame.jpg (one picture)."""

    def __init__(self, camera: OakMjpegCamera, port: int = 8080):
        self.camera = camera
        self.port = port
        self._httpd: Optional[ThreadingHTTPServer] = None

    def start(self) -> "MjpegServer":
        camera = self.camera

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:       # keep the console quiet
                pass

            def do_GET(self) -> None:
                if self.path in ("/", "/index.html"):
                    self._send(200, "text/html; charset=utf-8", PAGE)
                elif self.path.startswith("/frame.jpg"):
                    _, jpeg = camera.wait_frame(0, timeout=2.0)
                    if jpeg is None:
                        self._send(503, "text/plain", b"no picture from the camera yet")
                    else:
                        self._send(200, "image/jpeg", jpeg)
                elif self.path.startswith("/stream"):
                    self._stream()
                else:
                    self._send(404, "text/plain", b"not found")

            def _send(self, code: int, content_type: str, body: bytes) -> None:
                self.send_response(code)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _stream(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                last = 0
                try:
                    while camera.running:
                        last, jpeg = camera.wait_frame(last, timeout=2.0)
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass                                # the browser went away

        self._httpd = ThreadingHTTPServer(("", self.port), Handler)
        self._httpd.daemon_threads = True
        threading.Thread(target=self._httpd.serve_forever, name="mjpeg-http", daemon=True).start()
        return self

    def urls(self) -> list[str]:
        """Addresses to open in a browser."""
        urls = [f"http://{socket.gethostname()}.local:{self.port}/"]
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(("10.255.255.255", 1))        # sends nothing; picks the network interface
                urls.append(f"http://{s.getsockname()[0]}:{self.port}/")
        except OSError:
            pass
        urls.append(f"http://localhost:{self.port}/   (on the robot's own screen)")
        return urls

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    def __enter__(self) -> "MjpegServer":
        return self.start()

    def __exit__(self, *exc) -> None:
        self.stop()
