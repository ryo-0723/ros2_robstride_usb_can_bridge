#pragma once

#include "serial_can_bridge/serial_frame.hpp"

#include <can_msgs/msg/frame.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace serial_can_bridge
{
// RobStride USB-CAN AT mode: AT + packed ID (big endian) + DLC + data + CRLF.
// Supports extended data frames only. Throws std::invalid_argument for invalid
// IDs, DLC > 8, standard frames, RTR frames, or CAN error frames.
SerialFrame encode_frame(const can_msgs::msg::Frame& frame);

// Decode one AT packet after checking its length, DLC <= 8, flags and CRLF.
// Returns nullopt for a malformed frame. ROS header fields remain unset.
std::optional<can_msgs::msg::Frame> decode_frame(const SerialFrame& bytes);

// Stateful decoder for AT packets of 9 + DLC bytes (DLC 0..8).
// Use one instance per connection
// from one thread. Partial frames survive between calls; malformed bytes are
// skipped. ROS header fields are left for the caller to fill in.
class FrameDecoder
{
  public:
    std::vector<can_msgs::msg::Frame> decode(const std::vector<uint8_t>& bytes);
    // Discard a partial frame when the serial connection is reopened.
    void reset();

  private:
    SerialFrameParser parser_;
};
} // namespace serial_can_bridge
