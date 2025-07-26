import cv2
from http.server import BaseHTTPRequestHandler, HTTPServer

cap = cv2.VideoCapture(0)  # Open webcam (index 0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            ret, frame = cap.read()
            if ret:
                ret, jpg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                self.wfile.write(b'--frame\r\n')
                self.send_header('Content-type', 'image/jpeg')
                self.end_headers()
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b'\r\n')
            else:
                print("Failed to read frame")

HTTPServer(('', 8080), StreamingHandler).serve_forever()