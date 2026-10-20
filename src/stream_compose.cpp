#include "stream_compose.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace stream_compose
{

I420Frame::I420Frame(int w, int h, uint8_t y_val, uint8_t u_val, uint8_t v_val)
: width(w),
  height(h),
  y(static_cast<size_t>(w) * h, y_val),
  u(static_cast<size_t>(w / 2) * (h / 2), u_val),
  v(static_cast<size_t>(w / 2) * (h / 2), v_val)
{
}

namespace
{
void CopyPlaneSideBySide(
  const std::vector<uint8_t> & l, const std::vector<uint8_t> & r, int w, int h, std::vector<uint8_t> * out)
{
  out->resize(static_cast<size_t>(2) * w * h);
  for (int row = 0; row < h; ++row) {
    uint8_t * dst = out->data() + static_cast<size_t>(row) * 2 * w;
    std::memcpy(dst, l.data() + static_cast<size_t>(row) * w, w);
    std::memcpy(dst + w, r.data() + static_cast<size_t>(row) * w, w);
  }
}
}  // namespace

I420Frame ComposeSideBySide(const I420Frame & left, const I420Frame & right)
{
  I420Frame out;
  out.width = left.width * 2;
  out.height = left.height;
  out.timestamp = left.timestamp;
  CopyPlaneSideBySide(left.y, right.y, left.width, left.height, &out.y);
  CopyPlaneSideBySide(left.u, right.u, left.width / 2, left.height / 2, &out.u);
  CopyPlaneSideBySide(left.v, right.v, left.width / 2, left.height / 2, &out.v);
  return out;
}

std::optional<FramePairer::Pair> FramePairer::Push(int stream_index, I420Frame frame)
{
  const int self = stream_index == 0 ? 0 : 1;
  auto & other = pending_[1 - self];

  // Frames of the other lens that are too old to ever match are lost.
  while (!other.empty() && other.front().timestamp < frame.timestamp - tolerance_) {
    other.pop_front();
    ++dropped_;
  }
  if (!other.empty() && std::llabs(other.front().timestamp - frame.timestamp) <= tolerance_) {
    Pair pair;
    I420Frame partner = std::move(other.front());
    other.pop_front();
    if (self == 0) {
      pair.lens0 = std::move(frame);
      pair.lens1 = std::move(partner);
    } else {
      pair.lens0 = std::move(partner);
      pair.lens1 = std::move(frame);
    }
    return pair;
  }

  pending_[self].push_back(std::move(frame));
  if (pending_[self].size() > max_pending_) {
    pending_[self].pop_front();
    ++dropped_;
  }
  return std::nullopt;
}

struct H264Decoder::Impl
{
  AVCodecContext * ctx = nullptr;
  AVPacket * pkt = nullptr;
  AVFrame * frame = nullptr;
  SwsContext * sws = nullptr;
  int out_size = 0;
};

H264Decoder::H264Decoder(int out_size) : impl_(new Impl)
{
  impl_->out_size = out_size;
  const AVCodec * codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  impl_->ctx = avcodec_alloc_context3(codec);
  impl_->ctx->flags |= AV_CODEC_FLAG_LOW_DELAY;
  impl_->ctx->thread_count = 1;
  if (avcodec_open2(impl_->ctx, codec, nullptr) < 0) {
    avcodec_free_context(&impl_->ctx);
  }
  impl_->pkt = av_packet_alloc();
  impl_->frame = av_frame_alloc();
}

H264Decoder::~H264Decoder()
{
  sws_freeContext(impl_->sws);
  av_frame_free(&impl_->frame);
  av_packet_free(&impl_->pkt);
  avcodec_free_context(&impl_->ctx);
}

std::optional<I420Frame> H264Decoder::Decode(const uint8_t * data, size_t size, int64_t timestamp)
{
  if (!impl_->ctx) {
    return std::nullopt;
  }
  impl_->pkt->data = const_cast<uint8_t *>(data);
  impl_->pkt->size = static_cast<int>(size);
  impl_->pkt->pts = timestamp;
  if (avcodec_send_packet(impl_->ctx, impl_->pkt) < 0) {
    return std::nullopt;
  }
  std::optional<I420Frame> result;
  // Drain; with no B-frames there is at most one frame per packet.
  while (avcodec_receive_frame(impl_->ctx, impl_->frame) == 0) {
    const int w = impl_->out_size > 0 ? impl_->out_size : impl_->frame->width;
    const int h = impl_->out_size > 0 ? impl_->out_size : impl_->frame->height;
    I420Frame out(w, h);
    out.timestamp = timestamp;
    impl_->sws = sws_getCachedContext(
      impl_->sws, impl_->frame->width, impl_->frame->height, static_cast<AVPixelFormat>(impl_->frame->format), w, h,
      AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
    if (impl_->sws) {
      uint8_t * dst[4] = {out.y.data(), out.u.data(), out.v.data(), nullptr};
      int dst_stride[4] = {w, w / 2, w / 2, 0};
      sws_scale(impl_->sws, impl_->frame->data, impl_->frame->linesize, 0, impl_->frame->height, dst, dst_stride);
      result = std::move(out);
    }
    av_frame_unref(impl_->frame);
  }
  return result;
}

struct H264Encoder::Impl
{
  AVCodecContext * ctx = nullptr;
  AVFrame * frame = nullptr;
  AVPacket * pkt = nullptr;
  int64_t next_pts = 0;
};

H264Encoder::H264Encoder(int width, int height, int fps, int bitrate, int gop) : impl_(new Impl)
{
  const AVCodec * codec = avcodec_find_encoder_by_name("libx264");
  if (!codec) {
    return;
  }
  impl_->ctx = avcodec_alloc_context3(codec);
  impl_->ctx->width = width;
  impl_->ctx->height = height;
  impl_->ctx->time_base = AVRational{1, fps};
  impl_->ctx->framerate = AVRational{fps, 1};
  impl_->ctx->pix_fmt = AV_PIX_FMT_YUV420P;
  impl_->ctx->bit_rate = bitrate;
  impl_->ctx->gop_size = gop;
  impl_->ctx->max_b_frames = 0;
  impl_->ctx->thread_count = 2;  // N100 has 4 cores shared with recording and FAST-LIO2
  av_opt_set(impl_->ctx->priv_data, "preset", "ultrafast", 0);
  av_opt_set(impl_->ctx->priv_data, "tune", "zerolatency", 0);
  if (avcodec_open2(impl_->ctx, codec, nullptr) < 0) {
    avcodec_free_context(&impl_->ctx);
    return;
  }
  impl_->frame = av_frame_alloc();
  impl_->frame->format = AV_PIX_FMT_YUV420P;
  impl_->frame->width = width;
  impl_->frame->height = height;
  impl_->pkt = av_packet_alloc();
}

H264Encoder::~H264Encoder()
{
  av_frame_free(&impl_->frame);
  av_packet_free(&impl_->pkt);
  avcodec_free_context(&impl_->ctx);
}

bool H264Encoder::ok() const { return impl_->ctx != nullptr; }

bool H264Encoder::Encode(const I420Frame & frame, std::vector<uint8_t> * packet)
{
  packet->clear();
  if (!impl_->ctx || frame.width != impl_->ctx->width || frame.height != impl_->ctx->height) {
    return false;
  }
  AVFrame * f = impl_->frame;
  // The frame data points at `frame` for the duration of the call only.
  f->data[0] = const_cast<uint8_t *>(frame.y.data());
  f->data[1] = const_cast<uint8_t *>(frame.u.data());
  f->data[2] = const_cast<uint8_t *>(frame.v.data());
  f->linesize[0] = frame.width;
  f->linesize[1] = frame.width / 2;
  f->linesize[2] = frame.width / 2;
  f->pts = impl_->next_pts++;
  if (avcodec_send_frame(impl_->ctx, f) < 0) {
    return false;
  }
  while (avcodec_receive_packet(impl_->ctx, impl_->pkt) == 0) {
    packet->insert(packet->end(), impl_->pkt->data, impl_->pkt->data + impl_->pkt->size);
    av_packet_unref(impl_->pkt);
  }
  return true;
}

DualStreamCombiner::DualStreamCombiner(const CombinerConfig & config)
: config_(config),
  decoders_{H264Decoder(config.eye_size), H264Decoder(config.eye_size)},
  pairer_(config.pair_tolerance),
  encoder_(2 * config.eye_size, config.eye_size, config.fps, config.bitrate, config.gop)
{
}

bool DualStreamCombiner::PushPacket(
  int stream_index, const uint8_t * data, size_t size, int64_t timestamp, std::vector<uint8_t> * out,
  int64_t * out_timestamp)
{
  if (stream_index != 0 && stream_index != 1) {
    return false;
  }
  auto frame = decoders_[stream_index].Decode(data, size, timestamp);
  if (!frame) {
    return false;
  }
  auto pair = pairer_.Push(stream_index, std::move(*frame));
  if (!pair) {
    return false;
  }
  const I420Frame combined =
    config_.swap_lenses ? ComposeSideBySide(pair->lens1, pair->lens0) : ComposeSideBySide(pair->lens0, pair->lens1);
  *out_timestamp = pair->lens0.timestamp;
  return encoder_.Encode(combined, out) && !out->empty();
}

}  // namespace stream_compose
