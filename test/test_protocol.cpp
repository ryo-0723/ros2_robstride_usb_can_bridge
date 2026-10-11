#include "serial_can_bridge/protocol.hpp"

#include <gtest/gtest.h>
#include <array>
#include <stdexcept>

using serial_can_bridge::encode_frame;

TEST(Protocol, OfficialEl05Example)
{
    can_msgs::msg::Frame frame;
    frame.is_extended = true;
    frame.id = 0x1200fd01;
    frame.dlc = 8;
    frame.data = {5, 0x70, 0, 0, 1, 0, 0, 0};
    const serial_can_bridge::SerialFrame expected{
        0x41, 0x54, 0x90, 0x07, 0xe8, 0x0c, 8, 5, 0x70, 0, 0, 1, 0, 0, 0, 0x0d, 0x0a};
    EXPECT_EQ(encode_frame(frame), expected);
}

TEST(Protocol, FixedSizeEncodingPreservesBinaryData)
{
    can_msgs::msg::Frame frame;
    frame.is_extended = true;
    frame.id = 0x1fffffff;
    frame.dlc = 8;
    frame.data = {0x0d, 0x0a, 0x41, 0x54, 0, 0xff, 0x80, 0x01};
    const serial_can_bridge::SerialFrame expected{0x41, 0x54, 0xff, 0xff, 0xff, 0xfc,
                                                         8,    0x0d, 0x0a, 0x41, 0x54, 0,
                                                         0xff, 0x80, 0x01, 0x0d, 0x0a};
    EXPECT_EQ(encode_frame(frame), expected);
    frame.id = 0;
    EXPECT_EQ(encode_frame(frame)[5], 4);
    for (uint8_t dlc = 0; dlc < 8; ++dlc)
    {
        frame.dlc = dlc;
        EXPECT_THROW(encode_frame(frame), std::invalid_argument);
    }
}

TEST(Protocol, RejectsInvalidAndUnsupportedFrames)
{
    can_msgs::msg::Frame frame;
    frame.is_extended = true;
    frame.dlc = 8;
    frame.id = 0x20000000;
    EXPECT_THROW(encode_frame(frame), std::invalid_argument);
    frame.id = 0;
    frame.dlc = 9;
    EXPECT_THROW(encode_frame(frame), std::invalid_argument);
    frame.dlc = 8;
    frame.is_extended = false;
    EXPECT_THROW(encode_frame(frame), std::invalid_argument);
    frame.is_extended = true;
    frame.is_rtr = true;
    EXPECT_THROW(encode_frame(frame), std::invalid_argument);
    frame.is_rtr = false;
    frame.is_error = true;
    EXPECT_THROW(encode_frame(frame), std::invalid_argument);
}

using serial_can_bridge::FrameDecoder;

TEST(Protocol, DecodesOfficialExample)
{
    FrameDecoder decoder;
    const auto frames = decoder.decode(
        {0x41, 0x54, 0x90, 0x07, 0xe8, 0x0c, 8, 5, 0x70, 0, 0, 1, 0, 0, 0, 0x0d, 0x0a});
    ASSERT_EQ(frames.size(), 1U);
    EXPECT_EQ(frames[0].id, 0x1200fd01U);
    EXPECT_EQ(frames[0].dlc, 8);
    EXPECT_TRUE(frames[0].is_extended);
    EXPECT_FALSE(frames[0].is_rtr);
    EXPECT_FALSE(frames[0].is_error);
    EXPECT_EQ(frames[0].data, (std::array<uint8_t, 8>{5, 0x70, 0, 0, 1, 0, 0, 0}));
}

TEST(Protocol, DecodesEverySplitAndMultipleFramesWithBinaryDelimiters)
{
    for (const auto id : {0U, 0x1200fd01U, 0x1fffffffU})
    {
        const uint8_t dlc = 8;
        can_msgs::msg::Frame frame;
        frame.is_extended = true;
        frame.id = id;
        frame.dlc = dlc;
        frame.data = {0x0d, 0x0a, 0x41, 0x54, 0, 0xff, 0x80, 0x41};
        const auto packet = encode_frame(frame);
        for (std::size_t split = 0; split <= packet.size(); ++split)
        {
            FrameDecoder decoder;
            EXPECT_TRUE(decoder.decode({}).empty());
            auto frames = decoder.decode({packet.begin(), packet.begin() + split});
            auto rest = decoder.decode({packet.begin() + split, packet.end()});
            frames.insert(frames.end(), rest.begin(), rest.end());
            ASSERT_EQ(frames.size(), 1U);
            EXPECT_EQ(frames[0].id, id);
            EXPECT_EQ(frames[0].dlc, dlc);
            for (std::size_t i = 0; i < frame.data.size(); ++i)
            {
                EXPECT_EQ(frames[0].data[i], i < dlc ? frame.data[i] : 0);
            }
        }
        FrameDecoder decoder;
        std::vector<uint8_t> stream(packet.begin(), packet.end());
        stream.insert(stream.end(), packet.begin(), packet.end());
        EXPECT_EQ(decoder.decode(stream).size(), 2U);
        std::size_t count = 0;
        for (const auto byte : packet)
        {
            count += decoder.decode({byte}).size();
        }
        EXPECT_EQ(count, 1U);
    }
}

TEST(Protocol, SkipsNoiseInvalidDlcFlagsAndTrailerThenRecovers)
{
    can_msgs::msg::Frame frame;
    frame.is_extended = true;
    frame.id = 42;
    frame.dlc = 8;
    frame.data[0] = 0xff;
    const auto valid = encode_frame(frame);
    std::vector<uint8_t> stream(100000, 0);
    for (uint8_t flags = 0; flags < 8; ++flags)
    {
        if (flags == 4)
        {
            continue;
        }
        auto bad = valid;
        bad[5] = (bad[5] & 0xf8U) | flags;
        stream.insert(stream.end(), bad.begin(), bad.end());
    }
    auto bad = valid;
    bad[6] = 9;
    stream.insert(stream.end(), bad.begin(), bad.end());
    bad = valid;
    bad.back() = 0;
    stream.insert(stream.end(), bad.begin(), bad.end());
    stream.push_back(0x41); // Repeated A before a valid AT header.
    stream.insert(stream.end(), valid.begin(), valid.end());
    FrameDecoder decoder;
    const auto result = decoder.decode(stream);
    ASSERT_EQ(result.size(), 1U);
    EXPECT_EQ(result[0].id, 42U);
    EXPECT_EQ(result[0].data[0], 0xff);
}

TEST(Protocol, ResetDiscardsPartialFrame)
{
    const std::vector<uint8_t> packet{0x41, 0x54, 0, 0, 0, 4, 8,    0,   0,
                                      0,    0,    0, 0, 0, 0, 0x0d, 0x0a};
    FrameDecoder decoder;
    EXPECT_TRUE(decoder.decode({0x41}).empty());
    decoder.reset();
    EXPECT_TRUE(decoder.decode({packet.begin() + 1, packet.end()}).empty());
    EXPECT_EQ(decoder.decode(packet).size(), 1U);
}

TEST(Protocol, ValidatesOneFixedSizeFrame)
{
    using serial_can_bridge::decode_frame;
    const serial_can_bridge::SerialFrame valid{0x41, 0x54, 0, 0, 0, 4, 8,    0,   0,
                                                      0,    0,    0, 0, 0, 0, 0x0d, 0x0a};
    ASSERT_TRUE(decode_frame(valid));
    for (const std::size_t index : {0U, 1U, 5U, 6U, 15U, 16U})
    {
        auto invalid = valid;
        invalid[index] = 0;
        EXPECT_FALSE(decode_frame(invalid));
        std::vector<uint8_t> stream(invalid.begin(), invalid.end());
        stream.insert(stream.end(), valid.begin(), valid.end());
        FrameDecoder decoder;
        EXPECT_EQ(decoder.decode(stream).size(), 1U);
    }
    for (uint8_t dlc = 0; dlc < 8; ++dlc)
    {
        auto invalid = valid;
        invalid[6] = dlc;
        EXPECT_FALSE(decode_frame(invalid));
    }
}
