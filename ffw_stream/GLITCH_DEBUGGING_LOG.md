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

## Current Hypotheses & Next Steps

Since we have practically bulletproofed the UDP network transmission (massive socket buffers, jitter buffer, CBR tests, and MTU-aligned slicing), the root cause is highly likely **occurring before the stream even hits the network**.

### Hypothesis A: The Local `popen` Pipe is Losing Frame Boundary Sync
We are piping raw BGRA data from our C++ ZED loop directly into `ffmpeg` via `stdin` (a standard Linux pipe). 
- A 1080p BGRA frame is 8.2 Megabytes. 
- At 30 FPS, we are slamming 250 MB/s through a Linux pipe.
- If the CPU spikes or the pipe buffer fills up, `ffmpeg` might read a partial frame. Because `rawvideo` has no header, if `ffmpeg` reads 8.0 MB instead of 8.2 MB, the next frame's top pixels will become the current frame's bottom pixels! This perfectly creates a "cut in half, swapped" visual tear.

### Hypothesis B: The ZED SDK is Providing Corrupted Buffers
The Jetson Orin Nano is maxing out its resources to run the ZED SDK, IK solvers, and CPU video encoding simultaneously. The ZED SDK might occasionally drop a frame or yield a corrupted `sl::Mat` pointer to us.

**Next step for debugging**: We need to investigate replacing the raw `popen` pipe with a more robust memory-sharing mechanism (like AppSrc in a C++ GStreamer pipeline), or verifying that the raw byte count pushed to `ffmpeg` is perfectly locked and never drifts.
