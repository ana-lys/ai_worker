import socket
import struct
import numpy as np
import cv2
import select

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
        
        # If we received the last chunk of the frame, display it!
        # (Even if intermediate chunks were dropped by the network, we still display it to maintain framerate)
        if chunk_index == total_chunks - 1:
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
                    
                elif stream_type == 2: # IR (Y8)
                    frame_data = np.frombuffer(assembled_data, dtype=np.uint8)
                    img = frame_data.reshape((height, width))
                    cv2.imshow(f"{prefix} IR", img)
                    
            except Exception as e:
                print(f"Failed to display frame: {e}")
                
    cv2.waitKey(1) # Needs to be called to update OpenCV windows
