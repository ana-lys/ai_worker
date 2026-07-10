# FFW Unified Camera Streaming System

## Overview

This package provides a unified UDP H.264 streaming and receiving system for multiple camera sources:
- **D435**: RealSense RGB camera (hand/wrist mounted)
- **D405**: RealSense depth + IR pair (hand/wrist mounted, always dual)
- **ZED M**: Stereolabs stereo RGB camera
- **OAK-D Lite**: DepthAI RGB camera

The system uses a single launcher pair to manage all camera sources, with explicit selection of which RGB feed to stream.

---

## Quick Start

### Terminal 1: Start the Streamer

```bash
# For OAK-D Lite only (no D405s)
ros2 launch ffw_stream unified_stream_launch.py enable_d405s:=false rgb_source:=oakd_lite

# For D435 RGB + D405 hands (default)
ros2 launch ffw_stream unified_stream_launch.py

# For ZED M + D405 hands
ros2 launch ffw_stream unified_stream_launch.py rgb_source:=zedm
```

### Terminal 2: Start the Receiver

```bash
# Receiver auto-detects and displays all available streams
ros2 launch ffw_stream unified_receiver_launch.py
```

The receiver will:
- Open a window showing the RGB feed (if present)
- Show a placeholder message while waiting for the first stream packet
- Display whatever streams are actually streaming (no manual configuration needed)

---

## Architecture

### Streaming Path (Sender)

The **unified_stream_launch.py** launcher handles:

1. **RealSense Streamer** (`realsense_udp_streamer`):
   - Always streams D405 hand cameras (depth + IR) when enabled
   - Optional: also streams D435 RGB when `rgb_source:=d435`
   - Port mapping:
     - `base_port + 0, 1`: Camera 0 (depth, IR)
     - `base_port + 2, 3`: Camera 1 (depth, IR)
     - `base_port + 100`: D435 RGB (if enabled)

2. **ZED Streamer** (`zed_udp_streamer`):
   - Streams ZED M RGB to `base_port + 100` when `rgb_source:=zedm`
   - Uses NVIDIA JPEG library preload for hardware acceleration (Jetson)

3. **DepthAI Streamer** (`depthai_udp_streamer`):
   - Streams OAK-D Lite 4K RGB to `base_port + 100` when `rgb_source:=oakd_lite`
   - Native OAK-D H.264 encoder, zero CPU overhead on encode

### Receiving Path (Receiver)

The **unified_receiver_launch.py** launcher:

1. **Auto-discovery**: Listens on all known ports
   - `base_port + 0-3`: D405 depth/IR streams (if available)
   - `base_port + 100`: RGB stream (D435/ZED/OAK, whichever is active)
   - `base_port + 200`: Telemetry/status messages

2. **Display**:
   - Single unified dashboard showing all received streams
   - Automatic layout: RGB on top, depth/IR grids below
   - Shows placeholder text until first frame arrives
   - No configuration needed—displays whatever is streaming

---

## Launch Arguments

### Streamer (`unified_stream_launch.py`)

| Argument | Default | Description |
|----------|---------|-------------|
| `dest_ip` | `192.168.0.241` | Receiver IP address |
| `base_port` | `9000` | Base UDP port for all streams |
| `fps` | `30` | Frame rate for all cameras |
| `enable_d405s` | `true` | Enable D405 hand cameras |
| `rgb_source` | `d435` | RGB source: `d435`, `zedm`, or `oakd_lite` |

### Receiver (`unified_receiver_launch.py`)

No arguments—the receiver automatically detects and displays all streams on the default ports.

---

## Port Mapping Reference

All ports are relative to `base_port` (default: 9000):

```
base_port + 0   → Camera 0 Depth (RealSense)
base_port + 1   → Camera 0 IR (RealSense)
base_port + 2   → Camera 1 Depth (RealSense)
base_port + 3   → Camera 1 IR (RealSense)
base_port + 100 → Primary RGB (D435 / ZED / OAK)
base_port + 200 → Telemetry / Status
```

---

## Stream Encoding

All streams use **H.264 RTP over UDP** at the application layer:

- **Format**: Raw H.264 NAL units
- **Encapsulation**: RTP (RFC 3984)
- **Protocol**: UDP
- **Packet size**: 1316 bytes (optimized for IP fragmentation)
- **Bitrate**: ~30 Mbps (H.264 Main profile, keyframe every 1 sec)

---

## Codec Details

### D435 / D405 RGB (if applicable)
- Encoded via `ffmpeg -c:v libx264` (software)
- Preset: `ultrafast` (lowest latency)
- Tune: `zerolatency`

### ZED M
- Encoded via `ffmpeg -c:v libx264` with NVIDIA JPEG preload (Jetson)
- Same presets as D435

### OAK-D Lite
- Native OAK-D H.264 hardware encoder
- 4K (3840×2160) @ 30 fps
- Zero CPU overhead on the camera side

---

## Use Cases

### Test with OAK-D Only (No D405s)
```bash
# Terminal 1
ros2 launch ffw_stream unified_stream_launch.py enable_d405s:=false rgb_source:=oakd_lite

# Terminal 2
ros2 launch ffw_stream unified_receiver_launch.py
```
Result: Single RGB window with OAK-D 4K stream.

### Production Setup (D405 + D435)
```bash
# Terminal 1 (no args = defaults to D405 + D435)
ros2 launch ffw_stream unified_stream_launch.py

# Terminal 2
ros2 launch ffw_stream unified_receiver_launch.py
```
Result: Unified dashboard with D435 RGB on top, D405 depth/IR grids below.

### Switch Camera at Runtime
To change the RGB source without restarting:
```bash
# Terminal 1 - kill the old streamer (Ctrl+C)
# Then restart with new rgb_source
ros2 launch ffw_stream unified_stream_launch.py rgb_source:=zedm
```
The receiver will automatically detect the new stream and update the display.

---

## Files and Structure

### Launch Files
- `unified_stream_launch.py` - Main streamer entry point (selector for D435/ZED/OAK)
- `unified_receiver_launch.py` - Main receiver entry point (auto-detects streams)
- `receiver_launch.py` - Legacy receiver (kept for compatibility)

### Source Code
- `src/realsense_udp_streamer.cpp` - D435/D405 RGB/depth/IR encoder
- `src/realsense_udp_receiver.cpp` - Unified UDP receiver with OpenCV dashboard
- `src/zed_udp_streamer.cpp` - ZED M encoder
- `src/depthai_node.cpp` (in ffw_depthai) - OAK-D Lite encoder

### Configuration
- `config/camera_left_config.yaml` - D405 left camera settings (device ID, ports)
- `config/camera_right_config.yaml` - D405 right camera settings (device ID, ports)

---

## Known Limitations

1. **D405 serials are hardcoded**: Edit `config/*.yaml` to change device IDs
2. **D435 RGB streams only one camera**: The streamer detects D435 by serial and streams it as RGB; only one D435 per robot
3. **ZED/OAK selection is exclusive**: Cannot stream both ZED and OAK simultaneously (both use `base_port + 100`)
4. **Display is X11-only**: OpenCV windows require an X display server (no Wayland support yet)

---

## Troubleshooting

### Window doesn't appear
- Check `DISPLAY` environment variable: `echo $DISPLAY` (should show `:0` or `:1`)
- Ensure X server is running: `xdpyinfo -display :0` should succeed
- Window may be on another monitor/workspace

### Receiver shows placeholder forever
- Check streamer logs for startup errors
- Verify `dest_ip` in the launcher matches the receiver's IP
- Use `netstat -un | grep 9[0-2]` to see if UDP ports are receiving packets

### Glitchy or dropped frames
- Lower FPS: `fps:=15`
- Lower resolution (if applicable): configure in camera YAML files
- Check network path for congestion or packet loss

### GPU decode warnings
- GStreamer warnings about "Cannot query video position" are benign; they don't affect playback

---

## Future Improvements

- [ ] Auto-discovery of D435 serials via `librealsense2` instead of hardcoding
- [ ] Support simultaneous ZED + OAK (allocate different RGB ports)
- [ ] Wayland support for OpenCV windows
- [ ] ROS 2 Image topic publishers from the receiver
- [ ] Configurable receiver layout (horizontal/vertical stacking)

---

## Contact & Support

For issues, refer to the relevant source files or the parent `ai_worker` package documentation.
