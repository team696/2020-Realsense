import pyrealsense2 as rs
import cv2
import threading
from http.server import BaseHTTPRequestHandler,HTTPServer
from socketserver import ThreadingMixIn
from io import BytesIO
import numpy as np
from PIL import Image

pipeline = None

class CamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('.mjpg'):
            print("Got mjpeg request")
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while True:
                try:
                    #rc,img = capture.read()
                    #if not rc:
                    #    continue
                    frames = pipeline.wait_for_frames()
                    depth = frames.get_depth_frame().as_frame().get_data()
                    np_image = np.asanyarray(depth)
                    imgRGB=cv2.cvtColor(np_image,cv2.COLOR_GRAY2RGB)
                    print(type(imgRGB))
                    jpg = Image.fromarray(np_image)
                    tmpFile = BytesIO()
                    jpg = jpg.point(lambda i:i*(1./256)).convert('L')
                    jpg.save(tmpFile,'JPEG')
                    self.wfile.write("--jpgboundary".encode('utf-8'))
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(len(tmpFile.getvalue())))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG')
                    #time.sleep(0.05)
                except KeyboardInterrupt:
                    break
            return
        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>'.encode('utf-8'))
            self.wfile.write('<img src="cam.mjpg"/>'.encode('utf-8'))
            self.wfile.write('</body></html>'.encode('utf-8'))
            return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

def main():
    global pipeline
    pipeline = rs.pipeline()
    prof = pipeline.start()
    dev = prof.get_device()
    print(dev)
    print([str(x) for x in dev.query_sensors()])

    ds = rs.depth_sensor(dev.query_sensors()[0])
    depth_scale = ds.get_depth_scale() #metrics per 1 LSB
    depth_scale *= 39.3701 #Convert to inches

    try:
        server = ThreadedHTTPServer(('0.0.0.0', 8080), CamHandler)
        print("server started")
        server.serve_forever()
    except KeyboardInterrupt:
        pipeline.stop()
        server.socket.close()

if __name__ == '__main__':
    main()
