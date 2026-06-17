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

# Buffer to hold chunks for the current frames for each camera
# stream_type: 0=Color, 1=Depth, 2=IR
frame_buffers = {
    sock_left: {0: {}, 1: {}, 2: {}},
    sock_right: {0: {}, 1: {}, 2: {}}
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
        
        buffers = frame_buffers[s]
        
        if frame_id not in buffers[stream_type]:
            buffers[stream_type][frame_id] = {
                'chunks_received': 0,
                'total_chunks': total_chunks,
                'chunks': {},
                'width': width,
                'height': height,
            }
            
        buf = buffers[stream_type][frame_id]
        
        # Store chunk payload
        if chunk_index not in buf['chunks']:
            buf['chunks'][chunk_index] = payload
            buf['chunks_received'] += 1
        
        # If all chunks for this frame are received, display it
        if buf['chunks_received'] == buf['total_chunks']:
            # Assemble
            assembled_data = bytearray()
            for i in range(buf['total_chunks']):
                assembled_data.extend(buf['chunks'].get(i, b''))
                
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
                    # Clamp at 0.3 meters (3000 units)
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
                
            # Clean up this frame and any older frames to prevent memory leaks
            keys_to_delete = [fid for fid in buffers[stream_type].keys() if fid <= frame_id]
            for fid in keys_to_delete:
                del buffers[stream_type][fid]
                
    cv2.waitKey(1) # Needs to be called to update OpenCV windows
