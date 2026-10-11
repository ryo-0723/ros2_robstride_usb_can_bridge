#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace serial_can_bridge
{
using SerialFrame = std::vector<uint8_t>;

inline bool is_valid_serial_frame(const SerialFrame& bytes)
{
    return bytes.size() >= 9 && bytes[0] == 0x41 && bytes[1] == 0x54 && bytes[6] <= 8 &&
           bytes.size() == 9U + bytes[6] && (bytes[5] & 0x07U) == 0x04U &&
           bytes[bytes.size() - 2] == 0x0d && bytes.back() == 0x0a;
}

// Find packets of 9 + DLC bytes, using a bounded buffer for partial packets.
// Use one parser per connection, from one thread.
class SerialFrameParser
{
  public:
    std::vector<SerialFrame> feed(const uint8_t* bytes, std::size_t count)
    {
        std::vector<SerialFrame> frames;
        for (std::size_t offset = 0; offset < count; ++offset)
        {
            buffer_[size_++] = bytes[offset];
            while (size_ > 0)
            {
                const bool header_matches =
                    buffer_[0] == 0x41 && (size_ == 1 || buffer_[1] == 0x54);
                if (header_matches)
                {
                    if (size_ < 7)
                    {
                        break;
                    }
                    if (buffer_[6] <= 8 && (buffer_[5] & 0x07U) == 0x04U)
                    {
                        const std::size_t packet_size = 9U + buffer_[6];
                        if (size_ < packet_size)
                        {
                            break;
                        }
                        if (buffer_[packet_size - 2] == 0x0d && buffer_[packet_size - 1] == 0x0a)
                        {
                            frames.emplace_back(buffer_.begin(), buffer_.begin() + packet_size);
                            for (std::size_t i = packet_size; i < size_; ++i)
                            {
                                buffer_[i - packet_size] = buffer_[i];
                            }
                            size_ -= packet_size;
                            continue;
                        }
                    }
                }
                // Look for another AT header inside an invalid candidate.
                for (std::size_t i = 1; i < size_; ++i)
                {
                    buffer_[i - 1] = buffer_[i];
                }
                --size_;
            }
        }
        return frames;
    }

    void reset()
    {
        size_ = 0;
    }

  private:
    std::array<uint8_t, 17> buffer_{};
    std::size_t size_{0};
};
} // namespace serial_can_bridge
