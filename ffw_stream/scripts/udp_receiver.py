import cv2
import numpy as np
import threading
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstApp', '1.0')
from gi.repository import Gst, GstApp, GLib

Gst.init(None)

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
# Based on camera_left_config.yaml and camera_right_config.yaml
# We only receive what is being sent. If left Depth/IR are sent, we listen to those ports.
streams = {
    "Left RGB":   {"port": 8080, "type": "rgb"},
    "Left Depth": {"port": 8082, "type": "depth"},
    "Left IR":    {"port": 8084, "type": "ir"},
    
    "Right RGB":  {"port": 8081, "type": "rgb"},
    "Right Depth":{"port": 8083, "type": "depth"},
    "Right IR":   {"port": 8085, "type": "ir"}
}

# ---------------------------------------------------------
# Global Image Buffers
# ---------------------------------------------------------
latest_images = {k: None for k in streams.keys()}
lock = threading.Lock()

# ---------------------------------------------------------
# GStreamer Pipeline Builders
# ---------------------------------------------------------
def create_pipeline(name, info):
    port = info["port"]
    stype = info["type"]
    
    if stype == "rgb":
        # H264 decompressed to BGR
        pipe_str = (
            f"udpsrc port={port} ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! "
            f"rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink drop=true max-buffers=1"
        )
    elif stype == "ir":
        # H264 decompressed to Grayscale
        pipe_str = (
            f"udpsrc port={port} ! application/x-rtp, media=video, clock-rate=90000, encoding-name=H264 ! "
            f"rtph264depay ! decodebin ! videoconvert ! video/x-raw,format=GRAY8 ! appsink name=sink drop=true max-buffers=1"
        )
    elif stype == "depth":
        # Raw 16-bit payload disguised as UYVY
        pipe_str = (
            f"udpsrc port={port} ! application/x-rtp,media=video,clock-rate=90000,encoding-name=RAW,sampling=YCbCr-4:2:2,depth=8,width=480,height=270 ! "
            f"rtpvrawdepay ! appsink name=sink drop=true max-buffers=1"
        )
    
    pipeline = Gst.parse_launch(pipe_str)
    appsink = pipeline.get_by_name("sink")
    appsink.set_property("emit-signals", True)
    
    def on_new_sample(sink, name=name, stype=stype):
        sample = sink.emit("pull-sample")
        if not sample: return Gst.FlowReturn.OK
        
        buf = sample.get_buffer()
        caps = sample.get_caps()
        
        # Extract metadata
        struct = caps.get_structure(0)
        w = struct.get_value("width")
        h = struct.get_value("height")
        
        # Extract bytes
        _, map_info = buf.map(Gst.MapFlags.READ)
        data = map_info.data
        
        try:
            if stype == "rgb":
                img = np.ndarray((h, w, 3), buffer=data, dtype=np.uint8)
            elif stype == "ir":
                img = np.ndarray((h, w), buffer=data, dtype=np.uint8)
            elif stype == "depth":
                img = np.ndarray((h, w), buffer=data, dtype=np.uint16)
            
            with lock:
                latest_images[name] = img.copy()
        except Exception as e:
            pass
            
        buf.unmap(map_info)
        return Gst.FlowReturn.OK

    appsink.connect("new-sample", on_new_sample)
    return pipeline

# ---------------------------------------------------------
# Start Pipelines
# ---------------------------------------------------------
pipelines = []
for name, info in streams.items():
    p = create_pipeline(name, info)
    p.set_state(Gst.State.PLAYING)
    pipelines.append(p)

print("Listening for GStreamer UDP streams...")
print("Make sure you have python3-gst-1.0 installed.")

# ---------------------------------------------------------
# Main Display Loop
# ---------------------------------------------------------
import time
fps_trackers = {k: [0, time.time()] for k in streams.keys()}

while True:
    with lock:
        for name, img in latest_images.items():
            if img is not None:
                display_img = img
                
                # Apply colormap to depth
                if streams[name]["type"] == "depth":
                    # depth_scale converts raw Z16 to meters (default 0.001 = 1mm per unit)
                    depth_scale = 0.001
                    depth_meters = img.astype(np.float32) * depth_scale
                    max_display_meters = 1.0
                    img_clamped = np.clip(depth_meters, 0, max_display_meters)
                    img_normalized = (img_clamped / max_display_meters * 255).astype(np.uint8)
                    display_img = cv2.applyColorMap(img_normalized, cv2.COLORMAP_JET)
                
                cv2.imshow(name, display_img)
                latest_images[name] = None
                
                # Track FPS
                fps_trackers[name][0] += 1
                if fps_trackers[name][0] == 30:
                    elapsed = time.time() - fps_trackers[name][1]
                    fps = 30.0 / elapsed
                    print(f"[{name}] Receiver FPS: {fps:.1f} Hz")
                    fps_trackers[name] = [0, time.time()]

    key = cv2.waitKey(1)
    if key == 27 or key == ord('q'):
        break

for p in pipelines:
    p.set_state(Gst.State.NULL)
cv2.destroyAllWindows()
