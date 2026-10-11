#include "serial_can_bridge/protocol.hpp"

#include <algorithm>
#include <stdexcept>

namespace serial_can_bridge
{
SerialFrame encode_frame(const can_msgs::msg::Frame& frame)
{
    if (!frame.is_extended || frame.is_rtr || frame.is_error || frame.id > 0x1fffffffU ||
        frame.dlc != 8)
    {
        throw std::invalid_argument("Expected an extended CAN data frame, 29-bit ID, and DLC == 8");
    }

    // EL05 official manual, section 3.3.5: ID 0x1200FD01 -> 90 07 E8 0C.
    const uint32_t packed_id = (frame.id << 3) | 0x04U;
    SerialFrame bytes{};
    bytes[0] = 0x41; // A
    bytes[1] = 0x54; // T
    for (std::size_t i = 0; i < 4; ++i)
    {
        bytes[2 + i] = static_cast<uint8_t>(packed_id >> (24 - 8 * i));
    }
    bytes[6] = 8;
    std::copy(frame.data.begin(), frame.data.end(), bytes.begin() + 7);
    bytes[15] = 0x0d; // CR
    bytes[16] = 0x0a; // LF
    return bytes;
}

std::optional<can_msgs::msg::Frame> decode_frame(const SerialFrame& bytes)
{
    if (!is_valid_serial_frame(bytes))
    {
        return std::nullopt;
    }
    uint32_t packed_id = 0;
    for (std::size_t i = 2; i < 6; ++i)
    {
        packed_id = (packed_id << 8) | bytes[i];
    }
    can_msgs::msg::Frame frame;
    frame.id = packed_id >> 3;
    frame.is_extended = true;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.dlc = 8;
    std::copy_n(bytes.begin() + 7, 8, frame.data.begin());
    return frame;
}

std::vector<can_msgs::msg::Frame> FrameDecoder::decode(const std::vector<uint8_t>& bytes)
{
    std::vector<can_msgs::msg::Frame> frames;
    for (const auto& packet : parser_.feed(bytes.data(), bytes.size()))
    {
        frames.push_back(*decode_frame(packet));
    }
    return frames;
}

void FrameDecoder::reset()
{
    parser_.reset();
}
} // namespace serial_can_bridge
