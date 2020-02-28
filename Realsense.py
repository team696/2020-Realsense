import pyrealsense2 as rs
import cv2
import threading
from http.server import BaseHTTPRequestHandler,HTTPServer
from socketserver import ThreadingMixIn
from io import BytesIO
import numpy as np
from PIL import Image
import concurrent.futures
import time

TARGET_DISTANCE = 48
RANGE = 12

MIN_DISTANCE = TARGET_DISTANCE - (RANGE/2)
MAX_DISTANCE = MIN_DISTANCE + RANGE

def linear_map(x, inMin, inMax, outMin, outMax):
    # Figure out how 'wide' each range is
    inSpan = inMax - inMin
    outSpan = outMax - outMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(x - inMin) / float(inSpan)

    # Convert the 0-1 range into a value in the right range.
    return outMin + (valueScaled * outSpan)

def generate_colormap(greenRange):
    greenStart = round(128-(greenRange/2))
    greenStop = round(128+(greenRange/2))
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    #red -> green
    for i in range(0, greenStart):
        lut[i, 0, 0] = linear_map(i, 0, greenStart, 128, 0)
        lut[i, 0, 1] = linear_map(i, 0, greenStart, 0, 255)
    for i in range(greenStart, greenStop):
        lut[i, 0, :] = [0, 255, 0]
    for i in range(greenStop, 256):
        lut[i, 0, 1] = linear_map(i, greenStop, 255, 255, 0)
        lut[i, 0, 2] = linear_map(i, greenStop, 255, 0, 128)
    return lut

def processDepthImage(np_image, depth_scale, lut):
    np_image = np_image.astype(np.float)
    np_image *= depth_scale
    np_image = cv2.subtract(np_image, MIN_DISTANCE)
    np_image *= 255/(MAX_DISTANCE-MIN_DISTANCE)
    #np_image = cv2.convertScaleAbs(np_image)
    np_image = np.clip(np_image, 0, 255)
    np_image = np_image.astype(np.uint8)
    #print(np.amin(np_image), np.amax(np_image))
    imgRGB = cv2.applyColorMap(np_image, lut)
    return imgRGB

class Pipeline:
    def __init__(self):
        self.img = None
        self.producer_lock = threading.Lock()
        self.consumer_lock = threading.Lock()
        self.consumer_lock.acquire()

    def get_img(self, name):
        self.consumer_lock.acquire()
        img = self.img
        self.producer_lock.release()
        return img

    def set_img(self, img, name):
        self.producer_lock.acquire()
        self.img = img
        self.consumer_lock.release()

class RequestHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path.endswith('depth.mjpg'):
            self.send_response(200)
            self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
            self.end_headers()
            while True:
                try:
                    #frames = pipeline.wait_for_frames()
                    #depth = frames.get_depth_frame().as_frame().get_data()
                    #np_image = np.asanyarray(depth)
                    #print(np_image.shape, np_image.dtype)
                    #imgRGB = processDepthImage(np_image)
                    jpg = Image.fromarray(depthImg.get_img("Consumer"))
                    tmpFile = BytesIO()
                    #jpg = jpg.point(lambda i:i*(1./256)).convert('L')
                    jpg.save(tmpFile,'JPEG', quality=40)
                    self.wfile.write("--jpgboundary".encode('utf-8'))
                    self.send_header('Content-type','image/jpeg')
                    self.send_header('Content-length',str(len(tmpFile.getvalue())))
                    self.end_headers()
                    jpg.save(self.wfile,'JPEG', quality=40)
                    time.sleep(0.02)
                except KeyboardInterrupt:
                    break
            return

        if self.path.endswith('.html'):
            self.send_response(200)
            self.send_header('Content-type','text/html')
            self.end_headers()
            self.wfile.write('<html><head></head><body>'.encode('utf-8'))
            self.wfile.write('<img src="depth.mjpg"/>'.encode('utf-8'))
            #self.wfile.write('<img src="color.mjpg"/>'.encode('utf-8'))
            self.wfile.write('</body></html>'.encode('utf-8'))
            return


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

def camera_handler(depth_pipe):
    lut = generate_colormap(50)
    while True:
        try:
            print("Starting camera handler")
            pipeline = rs.pipeline()
            prof = pipeline.start()
            dev = prof.get_device()
            dev.hardware_reset()
            time.sleep(2)
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            pipeline = rs.pipeline()
            prof = pipeline.start(cfg)
            dev = prof.get_device()
            print(dev)
            print([str(x) for x in dev.query_sensors()])
    
            ds = rs.depth_sensor(dev.query_sensors()[0])
            depth_scale = ds.get_depth_scale() #metrics per 1 LSB
            depth_scale *= 39.3701 #Convert to inches
            print("%f inches per LSB"%depth_scale)
    
            try:
                while True:
                    starttime = time.time()
                    frames = pipeline.wait_for_frames()
                    depth = frames.get_depth_frame().as_frame().get_data()
                    np_image = np.asanyarray(depth)
                    #print(np_image.shape, np_image.dtype)
                    imgRGB = processDepthImage(np_image, depth_scale, lut)
                    depth_pipe.set_img(imgRGB, "Producer")
                    print("%f FPS"%(1./(time.time()-starttime)))
            except:
                print("Exception in camera handler.")
                pipeline.stop()
                raise
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(str(e))
            print("Restarting.")
            continue


depthImg = Pipeline()
def main():
    try:
        server = ThreadedHTTPServer(('0.0.0.0', 5809), RequestHandler)
        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            executor.submit(camera_handler, depthImg)
            executor.submit(server.serve_forever)
    except KeyboardInterrupt:
        server.socket.close()

if __name__ == '__main__':
    main()
