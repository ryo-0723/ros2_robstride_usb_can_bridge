#include "robstride_usb_can_bridge/protocol.hpp"

#include <gtest/gtest.h>
#include <array>
#include <stdexcept>

using robstride_usb_can_bridge::encode_frame;

TEST(Protocol, OfficialEl05Example)
{
    can_msgs::msg::Frame frame;
    frame.is_extended = true;
    frame.id = 0x1200fd01;
    frame.dlc = 8;
    frame.data = {5, 0x70, 0, 0, 1, 0, 0, 0};
    const robstride_usb_can_bridge::SerialFrame expected{
        0x41, 0x54, 0x90, 0x07, 0xe8, 0x0c, 8, 5, 0x70, 0, 0, 1, 0, 0, 0, 0x0d, 0x0a};
    EXPECT_EQ(encode_frame(frame), expected);
}

TEST(Protocol, EncodingPreservesBinaryData)
{
    can_msgs::msg::Frame frame;
    frame.is_extended = true;
    frame.id = 0x1fffffff;
    frame.dlc = 8;
    frame.data = {0x0d, 0x0a, 0x41, 0x54, 0, 0xff, 0x80, 0x01};
    const robstride_usb_can_bridge::SerialFrame expected{0x41, 0x54, 0xff, 0xff, 0xff, 0xfc,
                                                  8,    0x0d, 0x0a, 0x41, 0x54, 0,
                                                  0xff, 0x80, 0x01, 0x0d, 0x0a};
    EXPECT_EQ(encode_frame(frame), expected);
    frame.id = 0;
    EXPECT_EQ(encode_frame(frame)[5], 4);
    for (uint8_t dlc = 0; dlc < 8; ++dlc)
    {
        frame.dlc = dlc;
        const auto packet = encode_frame(frame);
        EXPECT_EQ(packet.size(), 9U + dlc);
        EXPECT_EQ(packet[6], dlc);
        EXPECT_EQ(packet[packet.size() - 2], 0x0d);
        EXPECT_EQ(packet.back(), 0x0a);
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

using robstride_usb_can_bridge::FrameDecoder;

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
        for (uint8_t dlc = 0; dlc <= 8; ++dlc)
        {
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
    // @note 正常なATヘッダーの直前にAが重複しても復帰できることを確認する
    stream.push_back(0x41);
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
    using robstride_usb_can_bridge::decode_frame;
    const robstride_usb_can_bridge::SerialFrame valid{0x41, 0x54, 0, 0, 0, 4, 8,    0,   0,
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

TEST(Protocol, VescDlc4UsesThirteenBytePacket)
{
    can_msgs::msg::Frame frame;
    frame.id = 0x300;
    frame.is_extended = true;
    frame.dlc = 4;
    frame.data = {0, 0, 0x0b, 0xb8, 0xff, 0xff, 0xff, 0xff};
    const robstride_usb_can_bridge::SerialFrame expected{0x41, 0x54, 0,    0,    0x18, 4,   4,
                                                  0,    0,    0x0b, 0xb8, 0x0d, 0x0a};
    EXPECT_EQ(encode_frame(frame), expected);
    const auto decoded = robstride_usb_can_bridge::decode_frame(expected);
    ASSERT_TRUE(decoded);
    EXPECT_EQ(decoded->id, 0x300U);
    EXPECT_EQ(decoded->dlc, 4);
    EXPECT_EQ(decoded->data, (std::array<uint8_t, 8>{0, 0, 0x0b, 0xb8, 0, 0, 0, 0}));
}

TEST(Protocol, MixedLengthsPreserveBoundaries)
{
    std::vector<uint8_t> stream;
    for (const uint8_t dlc : {8, 4, 0, 1, 7, 4, 8})
    {
        can_msgs::msg::Frame frame;
        frame.id = dlc;
        frame.is_extended = true;
        frame.dlc = dlc;
        frame.data = {0x41, 0x54, 0x0d, 0x0a, 0x41, 0x54, 0x0d, 0x0a};
        const auto packet = encode_frame(frame);
        stream.insert(stream.end(), packet.begin(), packet.end());
    }
    for (std::size_t split = 0; split <= stream.size(); ++split)
    {
        FrameDecoder decoder;
        auto frames = decoder.decode({stream.begin(), stream.begin() + split});
        const auto rest = decoder.decode({stream.begin() + split, stream.end()});
        frames.insert(frames.end(), rest.begin(), rest.end());
        ASSERT_EQ(frames.size(), 7U);
        const std::array<uint8_t, 7> lengths{8, 4, 0, 1, 7, 4, 8};
        for (std::size_t i = 0; i < frames.size(); ++i)
        {
            EXPECT_EQ(frames[i].id, lengths[i]);
            EXPECT_EQ(frames[i].dlc, lengths[i]);
        }
    }
}

TEST(Protocol, RejectsTruncatedAndOversizedPackets)
{
    const robstride_usb_can_bridge::SerialFrame valid{0x41, 0x54, 0,    0,    0,    4,   4,
                                               0x41, 0x54, 0x0d, 0x0a, 0x0d, 0x0a};
    for (std::size_t size = 0; size < valid.size(); ++size)
    {
        EXPECT_FALSE(robstride_usb_can_bridge::decode_frame({valid.begin(), valid.begin() + size}));
    }
    auto oversized = valid;
    oversized.push_back(0);
    EXPECT_FALSE(robstride_usb_can_bridge::decode_frame(oversized));
    auto bad = valid;
    bad.back() = 0;
    FrameDecoder decoder;
    EXPECT_TRUE(decoder.decode(bad).empty());
    EXPECT_EQ(decoder.decode(valid).size(), 1U);
}

TEST(Protocol, RecoveryKeepsBytesFollowingShortPacket)
{
    std::vector<uint8_t> stream{0x41, 0x54, 0, 0, 0, 4, 8};
    const robstride_usb_can_bridge::SerialFrame empty{0x41, 0x54, 0, 0, 0, 4, 0, 0x0d, 0x0a};
    const robstride_usb_can_bridge::SerialFrame short_packet{
        0x41, 0x54, 0, 0, 0x18, 4, 4, 0, 0, 0x0b, 0xb8, 0x0d, 0x0a};
    stream.insert(stream.end(), empty.begin(), empty.end());
    stream.insert(stream.end(), short_packet.begin(), short_packet.end());
    FrameDecoder decoder;
    const auto frames = decoder.decode(stream);
    ASSERT_EQ(frames.size(), 2U);
    EXPECT_EQ(frames[0].dlc, 0);
    EXPECT_EQ(frames[1].dlc, 4);
    EXPECT_EQ(frames[1].id, 0x300U);
}
