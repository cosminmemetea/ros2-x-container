#!/usr/bin/env python3
import cv2, time, threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler

DEVICE="/dev/video0"; WIDTH=720; HEIGHT=720; FPS=15; QUALITY=70
ENDPOINT="source_0"; BOUNDARY=b"--frame"

cv2.setNumThreads(1)
cap = cv2.VideoCapture(DEVICE, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

if not cap.isOpened():
    print(f"Error: Could not open {DEVICE}")
    raise SystemExit(1)

class State:
    def __init__(self): self.frame=None; self.lock=threading.Lock(); self.run=True
state = State()

def grabber():
    dt = 1.0/max(1,FPS)
    while state.run:
        ok, f = cap.read()
        if ok:
            with state.lock:
                state.frame = f
        time.sleep(dt)

threading.Thread(target=grabber, daemon=True).start()

def encode(frame):
    ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), QUALITY])
    return buf.tobytes() if ok else None

class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path != f"/{ENDPOINT}":
            self.send_error(404); return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        self.end_headers()
        try:
            last = 0
            while True:
                with state.lock:
                    f = None if state.frame is None else state.frame
                if f is None:
                    time.sleep(0.01); continue
                # Downscale a bit to save CPU:
                frame = cv2.resize(f, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA)
                jpg = encode(frame)
                if jpg is None: continue
                self.wfile.write(BOUNDARY + b"\r\n"
                                 b"Content-Type: image/jpeg\r\n"
                                 b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n")
                self.wfile.write(jpg); self.wfile.write(b"\r\n")
                # pace the writer
                time.sleep(1.0/max(1,FPS))
        except (BrokenPipeError, ConnectionResetError):
            pass
    def log_message(self, *a): return

print(f"Serving MJPEG on http://0.0.0.0:8080/{ENDPOINT}")
server = ThreadingHTTPServer(("", 8080), Handler)
try:
    server.serve_forever()
except KeyboardInterrupt:
    pass
finally:
    state.run=False; cap.release(); server.server_close()
