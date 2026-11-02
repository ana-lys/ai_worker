// depth_udp.hpp -- lossless 16-bit depth (raw Z16) over plain UDP.
//
// One depth frame = the D405's raw Z16 image (sensor units, depth_scale m/unit,
// 0 = no depth), optionally compressed, split into datagrams of at most
// kMaxDatagram bytes, each prefixed by DepthChunkHeader. No RTP, no DDS, no
// retransmits: the receiver keeps only the newest frame and drops a frame if
// any of its chunks is missing (same loss behaviour as the MJPEG feeds).
//
// Shared by realsense_udp_streamer (DepthSender) and realsense_udp_receiver
// (DepthReassembler) so the wire format cannot drift.
#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <zstd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace ffw_stream {
namespace depth_udp {

constexpr uint32_t kMagic = 0x31445A46;  // "FZD1" little-endian
constexpr size_t kMaxDatagram = 1400;    // fits a 1500 MTU with IP/UDP headers

// kZstdDelta (the default "zstd" mode): per row, left-neighbour delta ->
// zigzag -> high-byte plane then low-byte plane -> zstd. Exactly lossless; on
// noisy 0.1 mm D405 depth it compresses ~2x better than plain zstd and is
// faster (most of the entropy is in the low-byte plane, the high plane is
// almost all zeros).
enum class Codec : uint8_t { kRaw = 0, kZstd = 1, kZstdDelta = 2 };

// Forward filter: z16 (w*h) -> planes (2*w*h bytes).
inline void delta_planes_encode(const uint16_t *z, int w, int h, uint8_t *planes) {
  const size_t n = size_t(w) * size_t(h);
  uint8_t *hi = planes, *lo = planes + n;
  for (int y = 0; y < h; ++y) {
    uint16_t prev = 0;
    const uint16_t *row = z + size_t(y) * w;
    for (int x = 0; x < w; ++x) {
      const int16_t d = static_cast<int16_t>(static_cast<uint16_t>(row[x] - prev));
      prev = row[x];
      const uint16_t zz = static_cast<uint16_t>((static_cast<uint16_t>(d) << 1) ^ static_cast<uint16_t>(d >> 15));
      const size_t i = size_t(y) * w + x;
      hi[i] = static_cast<uint8_t>(zz >> 8);
      lo[i] = static_cast<uint8_t>(zz & 0xFF);
    }
  }
}

// Inverse filter: planes (2*w*h bytes) -> z16 (w*h).
inline void delta_planes_decode(const uint8_t *planes, int w, int h, uint16_t *z) {
  const size_t n = size_t(w) * size_t(h);
  const uint8_t *hi = planes, *lo = planes + n;
  for (int y = 0; y < h; ++y) {
    uint16_t prev = 0;
    uint16_t *row = z + size_t(y) * w;
    for (int x = 0; x < w; ++x) {
      const size_t i = size_t(y) * w + x;
      const uint16_t zz = static_cast<uint16_t>((hi[i] << 8) | lo[i]);
      const uint16_t d = static_cast<uint16_t>((zz >> 1) ^ static_cast<uint16_t>(-(zz & 1)));
      prev = static_cast<uint16_t>(prev + d);
      row[x] = prev;
    }
  }
}

#pragma pack(push, 1)
struct DepthChunkHeader {
  uint32_t magic;
  uint32_t frame_id;
  uint16_t chunk_idx;
  uint16_t chunk_count;
  uint16_t width;
  uint16_t height;
  uint8_t codec;         // Codec
  uint8_t reserved[3];
  float depth_scale;     // meters per Z16 unit
  uint32_t payload_size; // total (compressed) bytes of the frame
  uint64_t stamp_ns;     // sender CLOCK_REALTIME at capture
};
#pragma pack(pop)
static_assert(sizeof(DepthChunkHeader) == 36, "wire header must stay 36 bytes");

constexpr size_t kChunkPayload = kMaxDatagram - sizeof(DepthChunkHeader);

inline uint64_t realtime_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

// Sender: one per depth port. send() is called from the camera thread.
class DepthSender {
public:
  DepthSender() = default;
  DepthSender(const DepthSender &) = delete;
  DepthSender &operator=(const DepthSender &) = delete;
  ~DepthSender() { close_all(); }

  // zstd_level is ignored for Codec::kRaw. Returns false on socket error.
  bool open(const std::string &ip, int port, Codec codec, int zstd_level = 1) {
    close_all();
    codec_ = codec;
    level_ = zstd_level;
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ < 0) return false;
    // Blocking socket on purpose: if a frame burst outruns the send buffer
    // (net.core.wmem_max clamps this request, 208 KB stock) sendto waits
    // instead of silently dropping chunks -- i.e. natural pacing.
    int sndbuf = 4 * 1024 * 1024;
    setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    std::memset(&dst_, 0, sizeof(dst_));
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, ip.c_str(), &dst_.sin_addr) != 1) { close_all(); return false; }
    if (codec_ != Codec::kRaw) cctx_ = ZSTD_createCCtx();
    return true;
  }

  bool is_open() const { return sock_ >= 0; }

  // z16: width*height samples, row-contiguous. Returns bytes put on the wire
  // (payload only), 0 on failure.
  size_t send(const uint16_t *z16, int width, int height, float depth_scale, uint64_t stamp_ns) {
    if (sock_ < 0) return 0;
    const size_t raw_bytes = size_t(width) * size_t(height) * 2;
    const uint8_t *payload = reinterpret_cast<const uint8_t *>(z16);
    size_t payload_size = raw_bytes;
    Codec used = codec_;
    if (codec_ != Codec::kRaw && cctx_) {
      const void *src = z16;
      if (codec_ == Codec::kZstdDelta) {
        filt_.resize(raw_bytes);
        delta_planes_encode(z16, width, height, filt_.data());
        src = filt_.data();
      }
      comp_.resize(ZSTD_compressBound(raw_bytes));
      size_t n = ZSTD_compressCCtx(cctx_, comp_.data(), comp_.size(), src, raw_bytes, level_);
      if (!ZSTD_isError(n)) {
        payload = comp_.data();
        payload_size = n;
      } else {
        used = Codec::kRaw;  // fall back to raw for this frame
      }
    } else {
      used = Codec::kRaw;
    }

    const size_t chunks = (payload_size + kChunkPayload - 1) / kChunkPayload;
    if (chunks == 0 || chunks > 0xFFFF) return 0;
    DepthChunkHeader h{};
    h.magic = kMagic;
    h.frame_id = ++frame_id_;
    h.chunk_count = static_cast<uint16_t>(chunks);
    h.width = static_cast<uint16_t>(width);
    h.height = static_cast<uint16_t>(height);
    h.codec = static_cast<uint8_t>(used);
    h.depth_scale = depth_scale;
    h.payload_size = static_cast<uint32_t>(payload_size);
    h.stamp_ns = stamp_ns;

    dgram_.resize(kMaxDatagram);
    for (size_t c = 0; c < chunks; ++c) {
      const size_t off = c * kChunkPayload;
      const size_t len = std::min(kChunkPayload, payload_size - off);
      h.chunk_idx = static_cast<uint16_t>(c);
      std::memcpy(dgram_.data(), &h, sizeof(h));
      std::memcpy(dgram_.data() + sizeof(h), payload + off, len);
      if (sendto(sock_, dgram_.data(), sizeof(h) + len, 0,
                 reinterpret_cast<const sockaddr *>(&dst_), sizeof(dst_)) < 0) {
        // Destination unreachable etc.: drop the rest of this frame, keep going.
        return 0;
      }
    }
    return payload_size;
  }

private:
  void close_all() {
    if (sock_ >= 0) { ::close(sock_); sock_ = -1; }
    if (cctx_) { ZSTD_freeCCtx(cctx_); cctx_ = nullptr; }
  }

  int sock_ = -1;
  sockaddr_in dst_{};
  Codec codec_ = Codec::kZstd;
  int level_ = 1;
  ZSTD_CCtx *cctx_ = nullptr;
  uint32_t frame_id_ = 0;
  std::vector<uint8_t> comp_, dgram_, filt_;
};

// Receiver-side reassembly: feed every datagram to push(); when it returns
// true, frame()/width()/height()/depth_scale()/stamp_ns() hold a complete
// decoded Z16 frame. Older/incomplete frames are dropped when a newer frame
// starts (newest-frame-wins, like the leaky queues on the video feeds).
class DepthReassembler {
public:
  DepthReassembler() : dctx_(ZSTD_createDCtx()) {}
  ~DepthReassembler() { if (dctx_) ZSTD_freeDCtx(dctx_); }
  DepthReassembler(const DepthReassembler &) = delete;
  DepthReassembler &operator=(const DepthReassembler &) = delete;

  bool push(const uint8_t *data, size_t len) {
    if (len < sizeof(DepthChunkHeader)) return false;
    DepthChunkHeader h;
    std::memcpy(&h, data, sizeof(h));
    if (h.magic != kMagic || h.chunk_count == 0 || h.chunk_idx >= h.chunk_count) return false;
    const size_t plen = len - sizeof(h);
    const size_t off = size_t(h.chunk_idx) * kChunkPayload;
    if (off + plen > h.payload_size) return false;

    if (!active_ || h.frame_id != cur_.frame_id) {
      // Ignore stragglers of an older frame (wrap-safe compare).
      if (active_ && int32_t(h.frame_id - cur_.frame_id) < 0) return false;
      if (active_) ++dropped_;  // previous frame never completed
      cur_ = h;
      active_ = true;
      got_.assign(h.chunk_count, 0);
      received_ = 0;
      buf_.resize(h.payload_size);
    }
    if (got_[h.chunk_idx]) return false;
    got_[h.chunk_idx] = 1;
    ++received_;
    std::memcpy(buf_.data() + off, data + sizeof(h), plen);
    if (received_ != cur_.chunk_count) return false;

    active_ = false;
    const size_t raw_bytes = size_t(cur_.width) * size_t(cur_.height) * 2;
    frame_.resize(size_t(cur_.width) * size_t(cur_.height));
    if (cur_.codec == static_cast<uint8_t>(Codec::kZstd)) {
      size_t n = ZSTD_decompressDCtx(dctx_, frame_.data(), raw_bytes, buf_.data(), buf_.size());
      if (ZSTD_isError(n) || n != raw_bytes) { ++dropped_; return false; }
    } else if (cur_.codec == static_cast<uint8_t>(Codec::kZstdDelta)) {
      filt_.resize(raw_bytes);
      size_t n = ZSTD_decompressDCtx(dctx_, filt_.data(), raw_bytes, buf_.data(), buf_.size());
      if (ZSTD_isError(n) || n != raw_bytes) { ++dropped_; return false; }
      delta_planes_decode(filt_.data(), cur_.width, cur_.height, frame_.data());
    } else if (cur_.codec == static_cast<uint8_t>(Codec::kRaw)) {
      if (buf_.size() != raw_bytes) { ++dropped_; return false; }
      std::memcpy(frame_.data(), buf_.data(), raw_bytes);
    } else {
      ++dropped_;  // unknown codec (newer sender?)
      return false;
    }
    last_ = cur_;
    return true;
  }

  const std::vector<uint16_t> &frame() const { return frame_; }
  int width() const { return last_.width; }
  int height() const { return last_.height; }
  float depth_scale() const { return last_.depth_scale; }
  uint64_t stamp_ns() const { return last_.stamp_ns; }
  size_t payload_size() const { return last_.payload_size; }
  bool compressed() const { return last_.codec != static_cast<uint8_t>(Codec::kRaw); }
  Codec codec() const { return static_cast<Codec>(last_.codec); }
  uint64_t dropped() const { return dropped_; }

private:
  ZSTD_DCtx *dctx_ = nullptr;
  DepthChunkHeader cur_{}, last_{};
  bool active_ = false;
  std::vector<uint8_t> got_;
  size_t received_ = 0;
  std::vector<uint8_t> buf_, filt_;
  std::vector<uint16_t> frame_;
  uint64_t dropped_ = 0;
};

}  // namespace depth_udp
}  // namespace ffw_stream
