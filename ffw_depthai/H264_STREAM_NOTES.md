# H264 Stream Configuration Notes

## Pipeline Summary

The OAK-D camera produces **1920×1080 NV12** frames (hardcoded at `depthai_node.cpp:69`). These are fed to the MyriadX hardware H264 encoder, then pushed into a GStreamer pipeline that RTP-packetizes and sends over UDP to `192.168.0.241:9100`.

## Current H264 Parameters (`depthai_node.cpp:73-79`)

| Setting | Value |
|---------|-------|
| Profile | `H264_BASELINE` |
| Framerate | 30 fps |
| Bitrate | 16000 Kbps VBR |
| IDR interval | 30 frames (1s) |
| B-frames | 0 |

## Known Issue: Stream Looks Low-Res Despite Being 1080p

The pipeline *is* 1080p — if the receiver shows something smaller, the decoder or receiver pipeline is the problem, not the sender. But if the receiver shows 1080p that *looks* blurry/blocky, the likely causes are:

1. **CBR at 8 Mbps was marginal for H264 Baseline on MyriadX.** The MyriadX VPU hardware encoder is less efficient than x264 — Baseline profile uses CAVLC (not CABAC), costing ~10–15% efficiency. **Fixed 2026-07-21:** bitrate raised to 16 Mbps VBR, giving the encoder headroom on motion without wasting bits on static scenes.

2. **Chroma denoise was at 3** (`depthai_node.cpp:67`) — was smoothing color detail, making fine colored edges blur. **Fixed 2026-07-21:** lowered to 1.

3. **CBR's IDR-starvation issue is resolved by VBR.** The old CBR+1s keyframe combination starved surrounding frames — VBR allocates more bits to complex frames and fewer to simple ones.

### Tuning Levers (in likely order of impact)

| Fix | Change | Status |
|-----|--------|--------|
| **Switch to VBR** | `setRateControlMode(VBR)` + `setBitrateKbps(16000)` | ✅ **Applied 2026-07-21** — 16 Mbps VBR |
| ~~Raise bitrate~~ | ~~`setBitrateKbps(16000)`~~ | ✅ *Covered by CVBR change* |
| **Lower chroma denoise** | `setChromaDenoise(1)` or `0` | ✅ **Applied 2026-07-21** — set to 1 |
| **Raise sharpness** | `setSharpness(6)` or `8` | ✅ **Applied 2026-07-21** — set to 6 |

## H264_HIGH Profile — Do Not Use (Regression)

`H264_HIGH` was tried in place of `H264_BASELINE` and caused two problems:

- **High latency** — encoded frames accumulated delay in the pipeline.
- **Past-frame flicker** — old frames would visibly flash in the decoded stream, likely from a B-frame or reference-frame mismatch between the DepthAI encoder's GOP structure and what the GStreamer `rtph264pay` / receiver decoder expected.

The root cause is believed to be that **H264_HIGH enables B-frames by default** (and potentially multiple reference frames), which breaks the strictly-monotonic PTS order that the GStreamer pipeline and receiver decoder assume. The sender manually stamps PTS = DTS on every buffer (`depthai_node.cpp:232-233`), which is correct only for a P-frame-only (no B-frames) stream.

**Stick with `H264_BASELINE` + `setNumBFrames(0)`** — this guarantees a linear frame order with no frame reordering at the decoder, at the cost of slightly lower compression efficiency per bitrate.
