// rtp_capture_stream.hpp -- GStreamer RTP receive pipeline that recovers each
// frame's capture time from its RTP timestamp.
//
// realsense_udp_streamer pushes every frame with PTS = capture time (epoch ns,
// system clock) into rtpjpegpay / rtph264pay with timestamp-offset=0, so the
// RTP timestamp of every packet of a frame is capture time in 90 kHz ticks,
// mod 2^32 (wraps every 13.25 h; 11.1 us resolution). The receiver unwraps it
// against its own system clock -- valid because the robot follows the PC
// clock (chrony) and the capture is < 2 s old.
//
// How the time reaches the decoded frame:
//   - a probe on the depayloader sink pad remembers the RTP timestamp of each
//     incoming packet;
//   - a probe on the depayloader src pad fires synchronously when the frame
//     that packet completed is pushed, and rewrites the frame's PTS to the
//     unwrapped capture time;
//   - the decoder carries the PTS to the appsink sample.
// A sender that still uses a random RTP offset gives an implausible time ->
// the PTS is left alone and pull() reports capture_ns = 0 (caller falls back
// to arrival time).
#pragma once

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <gst/rtp/gstrtpbuffer.h>

#include <opencv2/core.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

namespace ffw_stream {

inline uint64_t system_now_ns() {
  return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
}

// 32-bit 90 kHz RTP timestamp -> epoch ns, choosing the wrap closest to now_ns.
inline uint64_t unwrap_rtp90k_ns(uint32_t rtp_ts, uint64_t now_ns) {
  const uint64_t now90 = gst_util_uint64_scale(now_ns, 90000, GST_SECOND);
  const int32_t d = static_cast<int32_t>(rtp_ts - static_cast<uint32_t>(now90));
  const uint64_t cap90 = static_cast<uint64_t>(static_cast<int64_t>(now90) + d);
  return gst_util_uint64_scale(cap90, GST_SECOND, 90000);
}

class RtpCaptureStream {
public:
  // Captures older/newer than this vs. the receiver clock are rejected
  // (wrong sender, unsynced clocks).
  static constexpr uint64_t kMaxSkewNs = 2ull * GST_SECOND;
  // PTS at/above this is an epoch capture time (year 2001+); below it is the
  // pipeline's own running time (no capture time recovered).
  static constexpr uint64_t kEpochFloorNs = 1000000000ull * GST_SECOND;

  RtpCaptureStream() = default;
  RtpCaptureStream(const RtpCaptureStream &) = delete;
  RtpCaptureStream &operator=(const RtpCaptureStream &) = delete;
  ~RtpCaptureStream() { close(); }

  // desc must name the depayloader "dep" and end in
  // "... ! video/x-raw,format=BGR|GRAY8 ! appsink name=sink ...".
  bool open(const std::string &desc, std::string *err) {
    close();
    GError *e = nullptr;
    pipe_ = gst_parse_launch(desc.c_str(), &e);
    if (e) {
      if (err) *err = e->message;
      g_error_free(e);
      close();
      return false;
    }
    sink_ = gst_bin_get_by_name(GST_BIN(pipe_), "sink");
    GstElement *dep = gst_bin_get_by_name(GST_BIN(pipe_), "dep");
    if (!sink_ || !dep) {
      if (err) *err = "pipeline needs elements named 'dep' and 'sink'";
      if (dep) gst_object_unref(dep);
      close();
      return false;
    }
    GstPad *in = gst_element_get_static_pad(dep, "sink");
    GstPad *out = gst_element_get_static_pad(dep, "src");
    gst_pad_add_probe(in, GST_PAD_PROBE_TYPE_BUFFER, &RtpCaptureStream::on_rtp, this, nullptr);
    gst_pad_add_probe(out, GST_PAD_PROBE_TYPE_BUFFER, &RtpCaptureStream::on_frame, this, nullptr);
    gst_object_unref(in);
    gst_object_unref(out);
    gst_object_unref(dep);
    if (gst_element_set_state(pipe_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      if (err) *err = "set_state(PLAYING) failed";
      close();
      return false;
    }
    return true;
  }

  // Waits up to timeout_ms for a decoded frame. On success fills `img`
  // (owning copy, CV_8UC3 for BGR / CV_8UC1 for GRAY8) and capture_ns
  // (0 = sender sent no usable capture time).
  bool pull(cv::Mat &img, uint64_t &capture_ns, int timeout_ms) {
    if (!sink_) return false;
    GstSample *s = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_),
                                                static_cast<GstClockTime>(timeout_ms) * GST_MSECOND);
    if (!s) return false;
    bool ok = false;
    GstBuffer *buf = gst_sample_get_buffer(s);
    GstStructure *st = gst_caps_get_structure(gst_sample_get_caps(s), 0);
    int w = 0, h = 0;
    gst_structure_get_int(st, "width", &w);
    gst_structure_get_int(st, "height", &h);
    const char *fmt = gst_structure_get_string(st, "format");
    const bool gray = fmt && std::string(fmt) == "GRAY8";
    GstMapInfo map;
    if (buf && w > 0 && h > 0 && gst_buffer_map(buf, &map, GST_MAP_READ)) {
      const size_t stride = GST_ROUND_UP_4(static_cast<size_t>(w) * (gray ? 1 : 3));
      if (map.size >= stride * static_cast<size_t>(h)) {
        cv::Mat view(h, w, gray ? CV_8UC1 : CV_8UC3, map.data, stride);
        img = view.clone();
        const GstClockTime pts = GST_BUFFER_PTS(buf);
        capture_ns = (GST_CLOCK_TIME_IS_VALID(pts) && pts >= kEpochFloorNs) ? pts : 0;
        ok = true;
      }
      gst_buffer_unmap(buf, &map);
    }
    gst_sample_unref(s);
    return ok;
  }

  // True once the pipeline posted an error or EOS (caller reopens).
  bool failed(std::string *msg = nullptr) {
    if (!pipe_) return true;
    GstBus *bus = gst_element_get_bus(pipe_);
    GstMessage *m = gst_bus_pop_filtered(
        bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    gst_object_unref(bus);
    if (!m) return false;
    if (msg) {
      if (GST_MESSAGE_TYPE(m) == GST_MESSAGE_ERROR) {
        GError *e = nullptr;
        gst_message_parse_error(m, &e, nullptr);
        *msg = e ? e->message : "error";
        if (e) g_error_free(e);
      } else {
        *msg = "EOS";
      }
    }
    gst_message_unref(m);
    return true;
  }

  // Frames whose RTP timestamp was not a plausible capture time.
  uint64_t rejected() const { return rejected_.load(); }

  void close() {
    if (pipe_) gst_element_set_state(pipe_, GST_STATE_NULL);
    if (sink_) gst_object_unref(sink_);
    if (pipe_) gst_object_unref(pipe_);
    sink_ = pipe_ = nullptr;
  }

private:
  static GstPadProbeReturn on_rtp(GstPad *, GstPadProbeInfo *info, gpointer self) {
    GstBuffer *b = GST_PAD_PROBE_INFO_BUFFER(info);
    GstRTPBuffer rtp = GST_RTP_BUFFER_INIT;
    if (b && gst_rtp_buffer_map(b, GST_MAP_READ, &rtp)) {
      static_cast<RtpCaptureStream *>(self)->last_rtp_.store(gst_rtp_buffer_get_timestamp(&rtp));
      gst_rtp_buffer_unmap(&rtp);
    }
    return GST_PAD_PROBE_OK;
  }

  static GstPadProbeReturn on_frame(GstPad *, GstPadProbeInfo *info, gpointer self) {
    auto *me = static_cast<RtpCaptureStream *>(self);
    const uint64_t now = system_now_ns();
    const uint64_t cap = unwrap_rtp90k_ns(me->last_rtp_.load(), now);
    const uint64_t skew = cap > now ? cap - now : now - cap;
    if (skew > kMaxSkewNs) {
      me->rejected_.fetch_add(1);
      return GST_PAD_PROBE_OK;
    }
    GstBuffer *b = gst_buffer_make_writable(GST_PAD_PROBE_INFO_BUFFER(info));
    GST_BUFFER_PTS(b) = cap;
    GST_PAD_PROBE_INFO_DATA(info) = b;
    return GST_PAD_PROBE_OK;
  }

  GstElement *pipe_ = nullptr;
  GstElement *sink_ = nullptr;
  std::atomic<uint32_t> last_rtp_{0};
  std::atomic<uint64_t> rejected_{0};
};

}  // namespace ffw_stream
