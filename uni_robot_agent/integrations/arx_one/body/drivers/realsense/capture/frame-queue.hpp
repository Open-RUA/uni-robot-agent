#pragma once
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace rua_helper {
struct Packet {
    std::uint64_t sequence=0;
    std::int64_t arrival=0;
    std::string metadata;
    std::vector<unsigned char> rgb,depth;
};

// One ordered history consumer; Slot::mutex protects all access. A returned
// packet is acknowledged by the next request's after_sequence, not by preview.
class FrameQueue {
    std::deque<std::shared_ptr<const Packet>> packets_;
public:
    static constexpr std::size_t capacity=8;
    std::uint64_t overflow_dropped=0,last_dropped_sequence=0;

    void push(std::shared_ptr<const Packet> packet) {
        if(packets_.size()==capacity) {
            last_dropped_sequence=packets_.front()->sequence;
            ++overflow_dropped;
            packets_.pop_front();
        }
        packets_.push_back(std::move(packet));
    }
    std::shared_ptr<const Packet> next_after(std::uint64_t after) {
        while(!packets_.empty() && packets_.front()->sequence<=after) packets_.pop_front();
        return packets_.empty()?nullptr:packets_.front();
    }
    std::size_t size() const { return packets_.size(); }
    std::uint64_t oldest() const { return packets_.empty()?0:packets_.front()->sequence; }
    std::uint64_t newest() const { return packets_.empty()?0:packets_.back()->sequence; }
    void clear() { packets_.clear(); overflow_dropped=0; last_dropped_sequence=0; }
};
} // namespace rua_helper
