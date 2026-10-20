// Hardware-independent helpers that turn the two H.264 preview streams of the
// X3 (stream_index 0 and 1, one fisheye lens each) into a single side-by-side
// H.264 stream. Depends on libavcodec/libswscale only (no camera SDK, no ROS).
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace stream_compose
{

// Planar YUV 4:2:0 picture (even width/height).
struct I420Frame
{
  int width = 0;
  int height = 0;
  std::vector<uint8_t> y, u, v;
  int64_t timestamp = 0;  // SDK timestamp of the source packet

  I420Frame() = default;
  I420Frame(int w, int h, uint8_t y_val = 0, uint8_t u_val = 128, uint8_t v_val = 128);
};

// Puts `left` and `right` (same size) next to each other: width doubles.
I420Frame ComposeSideBySide(const I420Frame & left, const I420Frame & right);

// Pairs frames of two streams by timestamp. A frame whose counterpart does not
// arrive within `tolerance` is dropped (never reused), so a lost frame costs one
// output frame and never desynchronises the two lenses.
class FramePairer
{
public:
  struct Pair
  {
    I420Frame lens0;
    I420Frame lens1;
  };

  explicit FramePairer(int64_t tolerance, size_t max_pending = 4) : tolerance_(tolerance), max_pending_(max_pending) {}

  std::optional<Pair> Push(int stream_index, I420Frame frame);
  size_t dropped() const { return dropped_; }

private:
  int64_t tolerance_;
  size_t max_pending_;
  std::deque<I420Frame> pending_[2];
  size_t dropped_ = 0;
};

// H.264 (Annex B) decoder that always outputs I420 scaled to `out_size` x
// `out_size`; out_size == 0 keeps the native size.
class H264Decoder
{
public:
  explicit H264Decoder(int out_size);
  ~H264Decoder();
  H264Decoder(const H264Decoder &) = delete;
  H264Decoder & operator=(const H264Decoder &) = delete;

  // Returns a frame once the decoder produced one (nothing before the first IDR).
  std::optional<I420Frame> Decode(const uint8_t * data, size_t size, int64_t timestamp);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Low-latency H.264 encoder: no B-frames, periodic IDR, SPS/PPS in every IDR so
// a bag that starts mid-stream is decodable from its first IDR.
class H264Encoder
{
public:
  H264Encoder(int width, int height, int fps, int bitrate, int gop);
  ~H264Encoder();
  H264Encoder(const H264Encoder &) = delete;
  H264Encoder & operator=(const H264Encoder &) = delete;

  bool ok() const;
  bool Encode(const I420Frame & frame, std::vector<uint8_t> * packet);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

struct CombinerConfig
{
  int eye_size = 1920;          // each lens is scaled to eye_size x eye_size
  bool swap_lenses = false;     // false: stream_index 0 on the left
  int64_t pair_tolerance = 20;  // in SDK timestamp units (ms-scale counter)
  int fps = 20;
  int bitrate = 20 * 1024 * 1024;
  int gop = 10;
};

// Two packet streams in, one side-by-side H.264 packet out.
class DualStreamCombiner
{
public:
  explicit DualStreamCombiner(const CombinerConfig & config);

  // Feeds one packet. Returns true and fills `out` when a side-by-side packet
  // was produced. `out_timestamp` is the SDK timestamp of the pair's lens 0.
  bool PushPacket(
    int stream_index, const uint8_t * data, size_t size, int64_t timestamp, std::vector<uint8_t> * out,
    int64_t * out_timestamp);

  size_t dropped_pairs() const { return pairer_.dropped(); }
  bool encoder_ok() const { return encoder_.ok(); }

private:
  CombinerConfig config_;
  H264Decoder decoders_[2];
  FramePairer pairer_;
  H264Encoder encoder_;
};

}  // namespace stream_compose
