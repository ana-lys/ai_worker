import socket
import struct
import numpy as np
import cv2
import select
import time

UDP_IP = "0.0.0.0" # Listen on all interfaces

# Set up socket for Left Camera (Port 8080)
sock_left = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_left.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024 * 10)
sock_left.bind((UDP_IP, 8080))

# Set up socket for Right Camera (Port 8081)
sock_right = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_right.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024 * 10)
sock_right.bind((UDP_IP, 8081))

sockets = [sock_left, sock_right]

# Persistent byte arrays to hold the raw frame data for robust assembly
persistent_buffers = {
    sock_left: {0: None, 1: None, 2: None},
    sock_right: {0: None, 1: None, 2: None}
}

fps_trackers = {} # key: (s, stream_type), value: [frames, start_time]
last_frame_id = {}

# 4(I) + 1(B) + 4(I) + 2(H) + 2(H) + 4(I) + 4(I) + 4(I) + 4(I) = 29 bytes
HEADER_FORMAT = '<IBIHHIIII'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)

print(f"Listening for RealSense streams on {UDP_IP}:8080 (Left) and {UDP_IP}:8081 (Right)...")

while True:
    # Use select to multiplex both sockets so we don't block on just one
    readable, _, _ = select.select(sockets, [], [], 0.01)
    
    for s in readable:
        try:
            packet, addr = s.recvfrom(65536)
        except BlockingIOError:
            continue
            
        if len(packet) < HEADER_SIZE:
            continue
            
        header_data = packet[:HEADER_SIZE]
        frame_id, stream_type, total_size, chunk_index, total_chunks, chunk_size, width, height, format_rs = struct.unpack(HEADER_FORMAT, header_data)
        
        payload = packet[HEADER_SIZE:]
        
        if persistent_buffers[s][stream_type] is None or len(persistent_buffers[s][stream_type]) != total_size:
            persistent_buffers[s][stream_type] = bytearray(total_size)
            
        start_idx = chunk_index * chunk_size
        end_idx = start_idx + len(payload)
        
        # Robustly copy payload directly into the persistent frame buffer
        persistent_buffers[s][stream_type][start_idx:end_idx] = payload
        
        # Track the last seen frame_id for this stream
        if s not in last_frame_id: last_frame_id[s] = {0:-1, 1:-1, 2:-1}
        
        # If the sender moved on to a NEW frame_id, it means the previous frame is completely finished sending!
        # This is the most bulletproof way to trigger a display update, guaranteeing it always fires even if the last chunk is dropped.
        if frame_id > last_frame_id[s][stream_type]:
            last_frame_id[s][stream_type] = frame_id
            
            assembled_data = persistent_buffers[s][stream_type]
            prefix = "Left" if s == sock_left else "Right"
                
            try:
                if stream_type == 0: # Color (RGB8)
                    frame_data = np.frombuffer(assembled_data, dtype=np.uint8)
                    img = frame_data.reshape((height, width, 3))
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # OpenCV expects BGR
                    cv2.imshow(f"{prefix} RGB", img)
                    
                elif stream_type == 1: # Depth (Z16)
                    img = np.frombuffer(assembled_data, dtype=np.uint16).reshape((height, width))
                    
                    # D405 default depth scale is usually 0.1mm per unit
                    MAX_DEPTH_UNITS = 3000
                    
                    img_clamped = np.clip(img, 0, MAX_DEPTH_UNITS)
                    img_normalized = (img_clamped / float(MAX_DEPTH_UNITS) * 255).astype(np.uint8)
                    img_color = cv2.applyColorMap(img_normalized, cv2.COLORMAP_JET)
                    
                    cv2.imshow(f"{prefix} Depth", img_color)
                    
                elif stream_type == 2: # IR (D405 often outputs 16-bit or UYVY for IR)
                    frame_data = np.frombuffer(assembled_data, dtype=np.uint8)
                    bpp = len(frame_data) // (height * width)
                    
                    if bpp == 1:
                        img = frame_data.reshape((height, width))
                    elif bpp == 2:
                        img_raw = frame_data.reshape((height, width, 2))
                        img = img_raw[:, :, 1]
                    elif bpp == 3:
                        img = frame_data.reshape((height, width, 3))
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    else:
                        img = np.zeros((height, width), dtype=np.uint8)
                        
                    cv2.imshow(f"{prefix} IR", img)
                    
            except Exception as e:
                # ignore silent errors from incomplete partial frames when initializing
                pass
                
            # FPS tracking
            tracker_key = (s, stream_type)
            if tracker_key not in fps_trackers:
                fps_trackers[tracker_key] = [0, time.time()]
            fps_trackers[tracker_key][0] += 1
            
            if fps_trackers[tracker_key][0] == 30:
                elapsed = time.time() - fps_trackers[tracker_key][1]
                fps = 30.0 / elapsed
                stream_name = "RGB" if stream_type == 0 else "Depth" if stream_type == 1 else "IR"
                print(f"[{prefix} {stream_name}] Receiver FPS: {fps:.1f} Hz")
                fps_trackers[tracker_key] = [0, time.time()]
                
    cv2.waitKey(1) # Needs to be called to update OpenCV windows
