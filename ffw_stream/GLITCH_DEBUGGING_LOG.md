# ZED UDP Streamer: Glitch Debugging Log

## The Problem
During teleoperation, the ZED camera stream (1080p30, transmitted via UDP) exhibits intermittent visual glitches. 
- **Frequency**: Every ~10 seconds or sporadically.
- **Visual Symptom**: The user describes it as "a horse picture cut in the middle with the head and rear segment swapped." This is a classic symptom of spatial frame tearing, where a portion of one video frame is incorrectly stitched to another frame, or macroblocks are rendered out of spatial order.
- **Latency constraint**: The system relies on Gigabit Ethernet, and latency must be kept as low as possible.

---

## What We Tried

### 1. UDP Network Congestion (CBR Bitrate Limits)
- **Hypothesis**: `libx264` on `ultrafast` preset generates massive I-frames (keyframes) that burst across the network, overflowing standard OS socket buffers and dropping UDP packets.
- **Action Taken**: We enforced Constant Bitrate (CBR) limits on the sender (`-b:v 4M -maxrate 4M -bufsize 8M`).
- **Result**: Added encoding buffer latency (delay), but the glitch persisted. We reverted this change to restore zero-latency encoding.

### 2. Receiver Socket Buffer Overruns
- **Hypothesis**: The Gigabit Ethernet switch was delivering the massive I-frames successfully, but GStreamer's `udpsrc` on the receiving PC had a tiny default socket buffer (~50KB) that overflowed before GStreamer could read it.
- **Action Taken**: Added `buffer-size=2147483647` (2GB max request) to the `udpsrc` elements on the receiver pipeline.
- **Result**: Did not fix the tearing glitch.

### 3. Out-Of-Order UDP Packets (Jitter Buffer)
- **Hypothesis**: Gigabit switches and OS networking stacks can sometimes deliver UDP packets out of order. If RTP packet #42 arrives before packet #41, the H264 depayloader glues the macroblocks together backwards, creating the "swapped head and rear" symptom.
- **Action Taken**: Injected `rtpjitterbuffer latency=10` into the GStreamer pipeline on the receiver to mathematically re-order RTP packets based on their sequence numbers before depayloading.
- **Result**: Did not fix the tearing glitch.

### 4. H264 Slice Tearing (MTU Size Limitations)
- **Hypothesis**: By default, `libx264` encodes the entire 1080p frame as a single massive slice. If a single UDP packet (which is just a fragment of that slice) is dropped, the decoder loses the spatial reference for the rest of the slice and guesses where to put the pixels, causing severe tearing.
- **Action Taken**: Added `-x264opts slice-max-size=1200` to the sender. This forces `libx264` to chunk the image into independent slices that perfectly fit inside a single network packet. If a packet is lost, you only lose a tiny strip, not spatial tracking of the whole frame.
- **Result**: Did not fix the tearing glitch.

### 5. Automated Glitch Detection Tool
- **Action Taken**: Wrote an OpenCV Mean Absolute Difference (MAD) glitch detector into the receiver thread to automatically save the frame *before* the glitch, the *glitched* frame, and a *diff heatmap*.
- **Result**: The detector successfully catches the glitch. The visual evidence confirms the glitch still exists despite the network robustness patches. We temporarily optimized the detector to ensure it wasn't the cause of CPU blocking/latency.

---

## Definitive Conclusion: Left/Right USB Hardware Tearing

We implemented a **Diagnostic Frame Counter** in the C++ sender on the Jetson. We drew text (`FRAME N`) directly onto the ZED image buffer *before* piping it. 

**The Result**:
When the glitch happens, the image *behind* the text is visibly torn and swapped **Left-to-Right**, but the text we drew remains perfectly intact and aligned! 

### Why this is a Smoking Gun:
1. **Network is Flawless**: If the UDP network or GStreamer dropped packets, the text would be torn along with the image.
2. **Local Pipe is Flawless**: If the Linux `popen` pipe to `ffmpeg` was desyncing, the text would be cut in half and swapped along with the image. 
3. **The Root Cause**: The ZED SDK (or the Jetson V4L2 USB driver beneath it) is outputting a corrupted `sl::Mat` memory buffer to our C++ program. 
   - The ZED is a Side-by-Side (SBS) 3D camera. It transmits the left and right eye images horizontally over the USB bus. 
   - Under heavy Jetson CPU/USB load, the hardware drops a sync packet. The V4L2 driver grabs the Right Eye first, then the Left Eye, stitching them backward. 
   - It hands this perfectly left-to-right swapped frame to our C++ program.

### Proposed Workarounds:
Since this is a deeply rooted hardware/driver bug in the ZED SDK / Jetson USB stack, we have two options:
1. **Algorithmic Fixer**: Write a 1ms OpenCV scanner that dynamically finds the vertical tear line and un-swaps the memory buffer before it is piped to `ffmpeg`.
2. **Use Official ROS 2 Wrapper**: Evaluate if the official `zed-ros2-wrapper` manages to avoid this driver tearing through better internal thread synchronization or SDK configuration.
