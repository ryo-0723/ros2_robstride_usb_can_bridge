#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace serial_can_bridge
{
using SerialFrame = std::array<uint8_t, 17>;

inline bool is_valid_serial_frame(const SerialFrame& bytes)
{
    return bytes[0] == 0x41 && bytes[1] == 0x54 && bytes[6] == 8 && (bytes[5] & 0x07U) == 0x04U &&
           bytes[15] == 0x0d && bytes[16] == 0x0a;
}

// Find fixed-size packets in a serial stream without allocating a partial buffer.
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
                    if (size_ < buffer_.size())
                    {
                        break;
                    }
                    if (is_valid_serial_frame(buffer_))
                    {
                        frames.push_back(buffer_);
                        size_ = 0;
                        break;
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
    SerialFrame buffer_{};
    std::size_t size_{0};
};
} // namespace serial_can_bridge
