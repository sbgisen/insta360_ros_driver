#include <gtest/gtest.h>

#include "../src/stream_compose.hpp"

using namespace stream_compose;

namespace
{
// Encodes `n` solid-colour frames of `size` x `size` and returns the packets.
std::vector<std::vector<uint8_t>> MakeStream(int size, uint8_t y, uint8_t u, uint8_t v, int n)
{
  H264Encoder enc(size, size, 20, 2000000, 5);
  std::vector<std::vector<uint8_t>> packets;
  for (int i = 0; i < n; ++i) {
    std::vector<uint8_t> p;
    if (enc.Encode(I420Frame(size, size, y, u, v), &p) && !p.empty()) {
      packets.push_back(p);
    }
  }
  return packets;
}
}  // namespace

TEST(Compose, SideBySideOrderAndSize)
{
  const I420Frame out = ComposeSideBySide(I420Frame(8, 4, 50), I420Frame(8, 4, 200));
  ASSERT_EQ(out.width, 16);
  ASSERT_EQ(out.height, 4);
  EXPECT_EQ(out.y[0], 50);
  EXPECT_EQ(out.y[7], 50);
  EXPECT_EQ(out.y[8], 200);
  EXPECT_EQ(out.y[3 * 16 + 15], 200);
}

TEST(Pairer, MatchesWithinToleranceAndDropsLoneFrames)
{
  FramePairer p(10);
  auto f = [](int64_t ts) {
    I420Frame fr(2, 2);
    fr.timestamp = ts;
    return fr;
  };
  EXPECT_FALSE(p.Push(0, f(100)));
  auto pair = p.Push(1, f(105));
  ASSERT_TRUE(pair);
  EXPECT_EQ(pair->lens0.timestamp, 100);
  EXPECT_EQ(pair->lens1.timestamp, 105);

  // lens 1 frame at 150 is lost; lens 0 frame 150 is stale once lens 1 at 200 arrives
  EXPECT_FALSE(p.Push(0, f(150)));
  EXPECT_FALSE(p.Push(1, f(200)));
  EXPECT_GE(p.dropped(), 1u);
  auto pair2 = p.Push(0, f(203));
  ASSERT_TRUE(pair2);
  EXPECT_EQ(pair2->lens1.timestamp, 200);
}

TEST(Combiner, EndToEndLeftRightAndSwap)
{
  const int eye = 32;
  const auto s0 = MakeStream(64, 60, 128, 128, 6);   // dark
  const auto s1 = MakeStream(64, 200, 128, 128, 6);  // bright
  ASSERT_EQ(s0.size(), s1.size());
  ASSERT_GE(s0.size(), 3u);

  for (bool swap : {false, true}) {
    CombinerConfig cfg;
    cfg.eye_size = eye;
    cfg.swap_lenses = swap;
    cfg.gop = 5;
    DualStreamCombiner comb(cfg);
    ASSERT_TRUE(comb.encoder_ok());
    H264Decoder dec(0);
    int decoded = 0;
    for (size_t i = 0; i < s0.size(); ++i) {
      std::vector<uint8_t> out;
      int64_t ts = 0;
      comb.PushPacket(0, s0[i].data(), s0[i].size(), 1000 + 50 * i, &out, &ts);
      if (comb.PushPacket(1, s1[i].data(), s1[i].size(), 1000 + 50 * i + 3, &out, &ts)) {
        EXPECT_EQ(ts, static_cast<int64_t>(1000 + 50 * i));
        auto fr = dec.Decode(out.data(), out.size(), ts);
        if (!fr) {
          continue;
        }
        ++decoded;
        EXPECT_EQ(fr->width, 2 * eye);
        EXPECT_EQ(fr->height, eye);
        const int left = fr->y[16 * 2 * eye + 8];
        const int right = fr->y[16 * 2 * eye + eye + 8];
        EXPECT_NEAR(left, swap ? 200 : 60, 8);
        EXPECT_NEAR(right, swap ? 60 : 200, 8);
      }
    }
    EXPECT_GE(decoded, 3);
  }
}

TEST(Combiner, MissingSecondStreamProducesNothing)
{
  const auto s0 = MakeStream(64, 60, 128, 128, 4);
  CombinerConfig cfg;
  cfg.eye_size = 32;
  DualStreamCombiner comb(cfg);
  for (size_t i = 0; i < s0.size(); ++i) {
    std::vector<uint8_t> out;
    int64_t ts;
    EXPECT_FALSE(comb.PushPacket(0, s0[i].data(), s0[i].size(), 1000 + 50 * i, &out, &ts));
  }
}
