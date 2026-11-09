#pragma once
#include <cstdint>
#include <cstring>

namespace rua_helper {
constexpr char kRequestMagic[8] = {'T','H','R','G','B','D','0','1'};
constexpr char kResponseMagic[8] = {'T','H','R','G','B','R','0','1'};
constexpr std::uint16_t kVersion = 1;
enum Operation : std::uint16_t { Hello=1, Acquire=2, Heartbeat=3, Read=4, Status=5, Release=6, ReadNext=7 };
enum Code : std::uint32_t { Ok=0, Invalid=1, Busy=2, NoFrame=3, NotNew=4, NotOwner=5, Internal=6, NotReady=7 };
#pragma pack(push, 1)
struct Request {
    char magic[8];
    std::uint16_t version, operation;
    std::uint32_t role_mask;
    std::uint64_t after_sequence, reserved;
};
struct Response {
    char magic[8];
    std::uint16_t version, operation;
    std::uint32_t code, metadata_bytes, rgb_bytes, depth_bytes, reserved;
};
#pragma pack(pop)
static_assert(sizeof(Request) == 32 && sizeof(Response) == 32, "wire header changed");
inline bool valid_request(const Request& r) {
    if (std::memcmp(r.magic, kRequestMagic, 8) || r.version != kVersion || r.reserved ||
        r.operation < Hello || r.operation > ReadNext) return false;
    if (r.operation == Acquire) return r.role_mask >= 1 && r.role_mask <= 65535 && r.after_sequence == 0;
    if (r.operation == Read || r.operation == ReadNext) return r.role_mask && r.role_mask <= 65535 && !(r.role_mask & (r.role_mask-1));
    return r.role_mask == 0 && r.after_sequence == 0;
}
} // namespace rua_helper
