// Ordinary-user Linux RGB-D worker. Binary wire v1 lives on inherited pipes;
// only ACQUIRE opens USB. No public socket, device reset, option or firmware API.
#include "capture-helper-wire.hpp"
#include "frame-queue.hpp"
#include <librealsense2/rs.hpp>
#include <poll.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/random.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
using namespace rua_helper;
constexpr int W=640, H=480, FPS=30;
constexpr std::size_t RGB_BYTES=W*H*3, DEPTH_BYTES=W*H*2;
constexpr std::int64_t SECOND=1000000000LL;
constexpr const char* CONTEXT=R"({"inherit":false,"device-mask":2,"dds":{"enabled":false}})";
// Public serial is the USB/ASIC identity already used by all camera clients.
// Linux librealsense SERIAL_NUMBER is a distinct firmware/module identity;
// bind by that SDK serial, then verify BOTH identities on the actual profile.
struct Role { std::string name, serial, sdk_serial; };
constexpr unsigned MAX_CAMERAS=16;
std::vector<Role> ROLES;
void add_camera(const std::string& value) {
    const auto a=value.find(','); const auto b=value.find(',',a==std::string::npos?0:a+1);
    if(a==std::string::npos || b==std::string::npos || !a || b==a+1 || b+1>=value.size() || value.find(',',b+1)!=std::string::npos)
        throw std::runtime_error("camera must be role,ASIC_serial,SDK_serial");
    Role role{value.substr(0,a),value.substr(a+1,b-a-1),value.substr(b+1)};
    for(const auto& r:ROLES) if(r.name==role.name || r.serial==role.serial || r.sdk_serial==role.sdk_serial)
        throw std::runtime_error("duplicate camera identity");
    if(ROLES.size()>=MAX_CAMERAS) throw std::runtime_error("at most 16 cameras per capture worker");
    ROLES.push_back(std::move(role));
}
volatile std::sig_atomic_t signalled=0;
void on_signal(int) { signalled=1; }
std::int64_t raw_ns() {
    timespec value{};
    if(clock_gettime(CLOCK_MONOTONIC_RAW,&value)) std::_Exit(70);
    return std::int64_t(value.tv_sec)*SECOND+value.tv_nsec;
}
std::int64_t realtime_ns() {
    timespec value{};
    if(clock_gettime(CLOCK_REALTIME,&value)) std::_Exit(70);
    return std::int64_t(value.tv_sec)*SECOND+value.tv_nsec;
}
std::string quote(const std::string& value) {
    std::ostringstream out; out << '"';
    for(unsigned char c:value) {
        if(c=='"' || c=='\\') out << '\\' << c;
        else if(c<32) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c) << std::dec;
        else out << c;
    }
    out << '"'; return out.str();
}
std::string frame_metadata(const rs2::frame& frame) {
    std::ostringstream out; out << '{';
    bool first=true;
    for(auto key:{RS2_FRAME_METADATA_FRAME_TIMESTAMP,RS2_FRAME_METADATA_SENSOR_TIMESTAMP,
                  RS2_FRAME_METADATA_ACTUAL_EXPOSURE,RS2_FRAME_METADATA_TIME_OF_ARRIVAL,
                  RS2_FRAME_METADATA_BACKEND_TIMESTAMP}) {
        if(!frame.supports_frame_metadata(key)) continue;
        if(!first) out << ',';
        first=false;
        out << quote(rs2_frame_metadata_to_string(key)) << ':'
            << quote(std::to_string(frame.get_frame_metadata(key)));
    }
    out << '}'; return out.str();
}
void diagnostic(const std::string& message) noexcept {
    // One bounded nonblocking write: diagnostics must not defeat the SDK or
    // shutdown watchdog if the Python owner's log consumer has stopped.
    try {
        const auto line=std::string("rua-linux-capture: ")+message.substr(0,1024)+"\n";
        const auto ignored=::write(STDERR_FILENO,line.data(),line.size()); (void)ignored;
    } catch(...) { }
}
template<std::size_t N> std::string array_json(const float (&v)[N]) {
    std::ostringstream out; out << std::setprecision(9) << '[';
    for(std::size_t i=0;i<N;++i) {
        if(!std::isfinite(v[i])) throw std::runtime_error("nonfinite calibration");
        if(i) out << ',';
        out << v[i];
    }
    out << ']'; return out.str();
}
std::string intrinsics_json(const rs2_intrinsics& v) {
    if(v.width!=W || v.height!=H || !std::isfinite(v.fx) || !std::isfinite(v.fy) ||
       !std::isfinite(v.ppx) || !std::isfinite(v.ppy) || v.fx<=0 || v.fy<=0 ||
       v.model<0 || v.model>=RS2_DISTORTION_COUNT) throw std::runtime_error("invalid intrinsics");
    std::ostringstream out; out << std::setprecision(9) << "{\"width\":" << W << ",\"height\":" << H
        << ",\"fx\":" << v.fx << ",\"fy\":" << v.fy << ",\"ppx\":" << v.ppx << ",\"ppy\":" << v.ppy
        << ",\"model\":" << int(v.model) << ",\"coeffs\":" << array_json(v.coeffs) << '}';
    return out.str();
}
void check_profile(const rs2::stream_profile& p,rs2_stream stream,rs2_format format) {
    auto v=p.as<rs2::video_stream_profile>();
    if(!v || p.stream_type()!=stream || p.format()!=format || p.fps()!=FPS || v.width()!=W || v.height()!=H)
        throw std::runtime_error("required 640x480@30 RGB8/Z16 profile unavailable");
}
std::string device_info(const rs2::device& d,rs2_camera_info key) {
    return d.supports(key)?d.get_info(key):"unavailable";
}
std::string calibration_json(const rs2::pipeline_profile& active,int role) {
    const auto d=active.get_device();
    if(device_info(d,RS2_CAMERA_INFO_SERIAL_NUMBER)!=ROLES[role].sdk_serial ||
       device_info(d,RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER)!=ROLES[role].serial ||
       device_info(d,RS2_CAMERA_INFO_NAME).find("405")==std::string::npos)
        throw std::runtime_error("actual camera identity mismatch");
    if(active.get_streams().size()!=2) throw std::runtime_error("unexpected stream count");
    const auto dp=active.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    const auto cp=active.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    check_profile(dp,RS2_STREAM_DEPTH,RS2_FORMAT_Z16); check_profile(cp,RS2_STREAM_COLOR,RS2_FORMAT_RGB8);
    const auto scale=d.first<rs2::depth_sensor>().get_depth_scale();
    if(!std::isfinite(scale) || scale<=0) throw std::runtime_error("invalid depth scale");
    const auto extr=dp.get_extrinsics_to(cp);
    // The configured device ID; frame names match the Python manifest (<device>/color_optical).
    const std::string id=ROLES[role].name;
    std::ostringstream out; out << std::setprecision(9) << "{\"depth_scale_m\":" << scale
        << ",\"depth_intrinsics\":" << intrinsics_json(dp.get_intrinsics())
        << ",\"rgb_intrinsics\":" << intrinsics_json(cp.get_intrinsics())
        << ",\"depth_to_rgb\":{\"rotation\":" << array_json(extr.rotation) << ",\"translation\":" << array_json(extr.translation) << '}'
        << ",\"depth_optical_frame_id\":" << quote(id+"/depth_optical") << ",\"rgb_optical_frame_id\":" << quote(id+"/color_optical")
        << ",\"firmware\":" << quote(device_info(d,RS2_CAMERA_INFO_FIRMWARE_VERSION))
        << ",\"sdk_serial\":" << quote(device_info(d,RS2_CAMERA_INFO_SERIAL_NUMBER))
        << ",\"asic_serial\":" << quote(device_info(d,RS2_CAMERA_INFO_ASIC_SERIAL_NUMBER))
        << ",\"sdk_version\":" << quote(RS2_API_VERSION_STR) << ",\"extrinsics_rotation_layout\":\"column_major\"}";
    return out.str();
}
struct Slot {
    std::mutex mutex;
    std::shared_ptr<const Packet> latest;
    FrameQueue history;
    std::string calibration,status="not_selected",last_error;
    std::uint64_t generation=0,attempts=0,frames=0,invalid_frames=0;
    std::int64_t first_arrival=0,last_arrival=0,max_gap=0,retry_at=0;
    std::atomic<std::int64_t> operation_deadline{0};
};
class Capture {
    std::uint32_t instance_,mask_;
    std::array<std::atomic<std::uint64_t>,MAX_CAMERAS>& counters_;
    std::thread coordinator_;
    // The official multi-camera sample shares one context but starts pipes in
    // sequence. Device-hub resolution can initialize devices before matching
    // serials: serialize SDK lifecycle, not callbacks, frame reads or backoff.
    std::timed_mutex sdk_lifecycle_;
    std::atomic<bool> stop_{false},done_{false};
    std::atomic<std::int64_t> context_deadline_{0},stop_deadline_{0};
    bool lock_lifecycle(std::unique_lock<std::timed_mutex>& lock,bool cleanup) {
        while(true) {
            if(stop_) {
                if(!cleanup) return false;
                const auto deadline=stop_deadline_.load();
                if(deadline && raw_ns()>deadline) {
                    diagnostic("shutdown lifecycle wait exceeded deadline; exiting without a hardware reset");
                    std::_Exit(124);
                }
            }
            if(lock.try_lock_for(std::chrono::milliseconds(50))) {
                if(stop_ && !cleanup) { lock.unlock(); return false; }
                return true;
            }
        }
    }
    void unavailable(int i,const std::string& status,const std::string& error="") {
        std::lock_guard<std::mutex> lock(slots[i].mutex);
        slots[i].latest.reset(); slots[i].history.clear(); slots[i].calibration.clear(); slots[i].status=status;
        if(!error.empty()) slots[i].last_error=error.substr(0,512);
    }
    void receive(int i,std::uint64_t generation,rs2::frame input) noexcept {
        const auto arrival=raw_ns();
        const auto arrival_realtime=realtime_ns();
        const auto paired_at=raw_ns();
        if(stop_) return;
        try {
            const auto frames=input.as<rs2::frameset>();
            if(!frames) return;
            const auto d=frames.get_depth_frame(); const auto c=frames.get_color_frame();
            if(!d || !c) return;
            check_profile(d.get_profile(),RS2_STREAM_DEPTH,RS2_FORMAT_Z16);
            check_profile(c.get_profile(),RS2_STREAM_COLOR,RS2_FORMAT_RGB8);
            const int ds=d.get_stride_in_bytes(),cs=c.get_stride_in_bytes();
            if(d.get_bytes_per_pixel()!=2 || c.get_bytes_per_pixel()!=3 || ds<W*2 || ds>W*4 || cs<W*3 || cs>W*6)
                throw std::runtime_error("invalid source stride");
            const auto dt=d.get_timestamp(),ct=c.get_timestamp();
            if(!std::isfinite(dt) || !std::isfinite(ct) || dt<0 || ct<0) throw std::runtime_error("invalid source timestamp");
            // These commissioned D405 streams run at the same 30 Hz. A frameset
            // is a container, not proof of temporal pairing.
            if(c.get_frame_number()!=d.get_frame_number())
                throw std::runtime_error("RGB/depth source frame numbers differ");
            if(!c.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP) ||
               !d.supports_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP))
                throw std::runtime_error("RGB/depth source exposure metadata missing");
            const auto cr=std::uint32_t(c.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP));
            const auto dr=std::uint32_t(d.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP));
            const auto skew=std::int32_t(cr-dr);
            if(std::abs(std::int64_t(skew))>20000)
                throw std::runtime_error("RGB/depth exposure skew exceeds 20 ms");
            auto packet=std::make_shared<Packet>(); packet->arrival=arrival;
            packet->rgb.resize(RGB_BYTES); packet->depth.resize(DEPTH_BYTES);
            const auto* db=static_cast<const unsigned char*>(d.get_data());
            const auto* cb=static_cast<const unsigned char*>(c.get_data());
            if(!db || !cb) throw std::runtime_error("missing source data");
            for(int y=0;y<H;++y) {
                std::memcpy(packet->rgb.data()+y*W*3,cb+y*cs,W*3);
                std::memcpy(packet->depth.data()+y*W*2,db+y*ds,W*2);
            }
            auto& slot=slots[i]; std::lock_guard<std::mutex> lock(slot.mutex);
            if(stop_ || generation!=slot.generation || slot.calibration.empty()) return;
            // A late callback cannot regress the host arrival/order contract.
            if(slot.last_arrival && arrival<=slot.last_arrival) return;
            packet->sequence=++counters_[i];
            const auto role=ROLES[i]; const std::string id=role.name;
            std::ostringstream out; out << std::setprecision(std::numeric_limits<double>::max_digits10)
                << "{\"device_id\":" << quote(id) << ",\"role\":" << quote(role.name) << ",\"serial\":" << quote(role.serial)
                << ",\"producer_instance\":" << quote(std::to_string(instance_)) << ",\"sequence\":" << quote(std::to_string(packet->sequence))
                << ",\"arrival_monotonic_ns\":" << quote(std::to_string(arrival)) << ",\"width\":640,\"height\":480"
                << ",\"timing\":{\"callback_realtime_ns\":" << quote(std::to_string(arrival_realtime))
                << ",\"callback_monotonic_raw_ns\":" << quote(std::to_string(arrival))
                << ",\"clock_pair_span_ns\":" << quote(std::to_string(paired_at-arrival)) << '}'
                << ",\"calibration\":" << slot.calibration << ",\"source\":{\"paired_in_source\":true,\"frameset_id\":"
                << quote(std::to_string(instance_)+":"+role.name+":"+std::to_string(packet->sequence))
                << ",\"stream_generation\":" << quote(std::to_string(generation))
                << ",\"rejected_pair_count\":" << slot.invalid_frames
                << ",\"last_rejected_pair_reason\":" << quote(slot.last_error)
                << ",\"rgb\":{\"frame_number\":" << quote(std::to_string(c.get_frame_number())) << ",\"timestamp_ms\":" << ct
                << ",\"timestamp_domain\":" << quote(rs2_timestamp_domain_to_string(c.get_frame_timestamp_domain()))
                << ",\"metadata\":" << frame_metadata(c) << '}'
                << ",\"depth\":{\"frame_number\":" << quote(std::to_string(d.get_frame_number())) << ",\"timestamp_ms\":" << dt
                << ",\"timestamp_domain\":" << quote(rs2_timestamp_domain_to_string(d.get_frame_timestamp_domain()))
                << ",\"metadata\":" << frame_metadata(d) << '}'
                << ",\"arrival_clock\":\"CLOCK_MONOTONIC_RAW\",\"arrival_semantics\":\"sdk_frameset_callback_entry\",\"cross_camera_hardware_sync\":false}}";
            packet->metadata=out.str();
            if(packet->metadata.size()>65536) throw std::runtime_error("metadata too large");
            if(!slot.first_arrival) slot.first_arrival=arrival;
            if(slot.last_arrival) slot.max_gap=std::max(slot.max_gap,arrival-slot.last_arrival);
            slot.last_arrival=arrival; ++slot.frames; slot.latest=packet; slot.history.push(packet); slot.status="streaming";
        } catch(const std::exception& error) {
            auto& slot=slots[i]; std::lock_guard<std::mutex> lock(slot.mutex);
            ++slot.invalid_frames; slot.last_error=std::string(error.what()).substr(0,512);
        }
    }
    void worker(rs2::context& context,int i) noexcept {
        auto& slot=slots[i]; unsigned failures=0;
        while(!stop_) {
            std::unique_ptr<rs2::pipeline> pipeline;
            std::unique_lock<std::timed_mutex> sdk_lock(sdk_lifecycle_,std::defer_lock);
            bool started=false; std::int64_t streaming_at=0;
            try {
                unavailable(i,"waiting_sdk_lifecycle");
                if(!lock_lifecycle(sdk_lock,false)) break;
                unavailable(i,"starting"); std::uint64_t generation;
                {
                    std::lock_guard<std::mutex> lock(slot.mutex);
                    generation=++slot.generation; ++slot.attempts; slot.retry_at=0;
                }
                slot.operation_deadline=raw_ns()+30*SECOND;
                pipeline=std::make_unique<rs2::pipeline>(context);
                rs2::config config; config.enable_device(ROLES[i].sdk_serial);
                config.enable_stream(RS2_STREAM_COLOR,W,H,RS2_FORMAT_RGB8,FPS);
                config.enable_stream(RS2_STREAM_DEPTH,W,H,RS2_FORMAT_Z16,FPS);
                auto active=pipeline->start(config,[this,i,generation](rs2::frame frame){receive(i,generation,std::move(frame));});
                started=true;
                auto calibration=calibration_json(active,i);
                streaming_at=raw_ns();
                {
                    std::lock_guard<std::mutex> lock(slot.mutex);
                    slot.calibration=std::move(calibration); slot.status="awaiting_frames";
                }
                slot.operation_deadline=0;
                sdk_lock.unlock();
                while(!stop_) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    std::int64_t latest;
                    { std::lock_guard<std::mutex> lock(slot.mutex); latest=slot.latest?slot.latest->arrival:streaming_at; }
                    if(raw_ns()-latest>3*SECOND) throw std::runtime_error("no valid paired frames for 3 seconds");
                    // A durable recovery resets exponential backoff; flapping does not.
                    if(raw_ns()-streaming_at>10*SECOND) failures=0;
                }
            } catch(const std::exception& error) {
                unavailable(i,"recovering",error.what()); diagnostic(std::string(ROLES[i].name)+": "+error.what());
            } catch(...) { unavailable(i,"recovering","unknown SDK error"); }
            if(pipeline) {
                if(!sdk_lock.owns_lock()) lock_lifecycle(sdk_lock,true);
                // Waiting for the SDK lock is not an SDK operation timeout.
                // Owner shutdown has its separate process-wide 5-second bound.
                slot.operation_deadline=raw_ns()+5*SECOND;
                if(started) {
                    try { pipeline->stop(); } catch(const std::exception& error) { diagnostic(error.what()); }
                }
                pipeline.reset(); // destructor is included in the operation watchdog
            }
            slot.operation_deadline=0;
            if(sdk_lock.owns_lock()) sdk_lock.unlock();
            unavailable(i,stop_?"stopped":"retry_wait");
            if(stop_) break;
            const auto delay=std::min(0.5*double(1u<<std::min(failures++,4u)),8.0);
            const auto retry_at=raw_ns()+static_cast<std::int64_t>(delay*SECOND);
            { std::lock_guard<std::mutex> lock(slot.mutex); slot.retry_at=retry_at; }
            while(!stop_ && raw_ns()<retry_at) std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
    void run() noexcept {
        try {
            rs2_error* err=nullptr; const auto runtime=rs2_get_api_version(&err); rs2::error::handle(err);
            if(runtime!=RS2_API_VERSION) throw std::runtime_error("runtime SDK version differs from compile SDK");
            rs2::log_to_callback(RS2_LOG_SEVERITY_WARN,[](rs2_log_severity,const rs2::log_message& message) noexcept {
                try { diagnostic(message.raw()); } catch(...) { }
            });
            rs2::context context(CONTEXT);
            context_deadline_=0;
            std::array<std::thread,MAX_CAMERAS> workers;
            try {
                for(unsigned i=0;i<ROLES.size();++i) if(mask_&(1u<<i)) workers[i]=std::thread([this,&context,i]{worker(context,i);});
            } catch(...) {
                stop();
                for(auto& thread:workers) if(thread.joinable()) thread.join();
                throw;
            }
            for(auto& thread:workers) if(thread.joinable()) thread.join();
            context_deadline_=raw_ns()+5*SECOND; // includes context destruction below
        } catch(const std::exception& error) {
            for(unsigned i=0;i<ROLES.size();++i) if(mask_&(1u<<i)) unavailable(i,"error",error.what());
            diagnostic(error.what());
        }
        context_deadline_=0; done_=true;
    }
public:
    std::array<Slot,MAX_CAMERAS> slots;
    Capture(std::uint32_t instance,std::array<std::atomic<std::uint64_t>,MAX_CAMERAS>& counters,std::uint32_t mask)
        :instance_(instance),mask_(mask),counters_(counters) {
        context_deadline_=raw_ns()+30*SECOND;
        coordinator_=std::thread([this]{run();});
    }
    void stop() {
        if(!stop_.exchange(true)) {
            stop_deadline_=raw_ns()+5*SECOND;
            for(unsigned i=0;i<ROLES.size();++i) if(mask_&(1u<<i)) unavailable(i,"stopping");
        }
    }
    bool done() const { return done_; }
    bool stopping() const { return stop_; }
    std::uint32_t mask() const { return mask_; }
    bool deadline_expired() const {
        const auto now=raw_ns();
        const auto c=context_deadline_.load(),s=stop_deadline_.load();
        if((c && now>c) || (!done_ && s && now>s)) return true;
        for(const auto& slot:slots) { const auto d=slot.operation_deadline.load(); if(d && now>d) return true; }
        return false;
    }
    std::string status() {
        std::ostringstream out; out << std::setprecision(9) << "{\"state\":" << quote(done_?"stopped":stop_?"stopping":"running")
            << ",\"role_mask\":" << mask_ << ",\"producer_instance\":" << quote(std::to_string(instance_)) << ",\"roles\":[";
        for(unsigned i=0;i<ROLES.size();++i) {
            auto& s=slots[i]; std::lock_guard<std::mutex> lock(s.mutex); if(i) out << ',';
            const auto age=s.latest?double(raw_ns()-s.latest->arrival)/1e9:-1.0;
            const auto fps=s.frames>1 && s.last_arrival>s.first_arrival?double(s.frames-1)*1e9/double(s.last_arrival-s.first_arrival):0.0;
            out << "{\"role\":" << quote(ROLES[i].name) << ",\"serial\":" << quote(ROLES[i].serial)
                << ",\"sdk_serial\":" << quote(ROLES[i].sdk_serial)
                << ",\"status\":" << quote(s.status) << ",\"sequence\":" << quote(std::to_string(counters_[i].load()))
                << ",\"stream_generation\":" << quote(std::to_string(s.generation)) << ",\"start_attempts\":" << s.attempts
                << ",\"source_frames\":" << s.frames << ",\"invalid_frames\":" << s.invalid_frames
                << ",\"source_fps_since_acquire\":" << fps << ",\"max_source_gap_ms\":" << double(s.max_gap)/1e6
                << ",\"queue_capacity\":" << FrameQueue::capacity << ",\"queue_retained\":" << s.history.size()
                << ",\"queue_overflow_dropped\":" << s.history.overflow_dropped
                << ",\"queue_last_dropped_sequence\":" << quote(std::to_string(s.history.last_dropped_sequence))
                << ",\"frame_age_s\":" << (age>=0?std::to_string(age):"null")
                << ",\"retry_in_s\":" << double(std::max<std::int64_t>(0,s.retry_at-raw_ns()))/1e9
                << ",\"last_error\":" << quote(s.last_error) << '}';
        }
        out << "]}"; return out.str();
    }
    ~Capture() { stop(); if(coordinator_.joinable()) coordinator_.join(); }
};
bool transfer(int fd,void* data,std::size_t bytes,bool sending,std::int64_t deadline) {
    auto* cursor=static_cast<unsigned char*>(data);
    while(bytes) {
        if(signalled || raw_ns()>deadline) return false;
        const auto count=sending ? ::write(fd,cursor,bytes) : ::read(fd,cursor,bytes);
        if(count>0) { cursor+=count; bytes-=std::size_t(count); continue; }
        if(count==0) return false;
        if(errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) return false;
        pollfd p{fd,short(sending?POLLOUT:POLLIN),0};
        if(poll(&p,1,50)<0 && errno!=EINTR) return false;
    }
    return true;
}
bool respond(std::uint16_t operation,Code code,const std::string& metadata,std::shared_ptr<const Packet> packet={}) {
    if(metadata.size()>65536) return false;
    Response header{}; std::memcpy(header.magic,kResponseMagic,8); header.version=kVersion; header.operation=operation;
    header.code=code; header.metadata_bytes=std::uint32_t(metadata.size());
    if(packet) { header.rgb_bytes=RGB_BYTES; header.depth_bytes=DEPTH_BYTES; }
    const auto deadline=raw_ns()+2*SECOND;
    if(!transfer(STDOUT_FILENO,&header,sizeof(header),true,deadline) ||
       !transfer(STDOUT_FILENO,const_cast<char*>(metadata.data()),metadata.size(),true,deadline)) return false;
    return !packet || (transfer(STDOUT_FILENO,const_cast<unsigned char*>(packet->rgb.data()),packet->rgb.size(),true,deadline) &&
                       transfer(STDOUT_FILENO,const_cast<unsigned char*>(packet->depth.data()),packet->depth.size(),true,deadline));
}
int serve(bool offline) {
    const auto stderr_flags=fcntl(STDERR_FILENO,F_GETFL);
    if(stderr_flags>=0) fcntl(STDERR_FILENO,F_SETFL,stderr_flags|O_NONBLOCK);
    if(!offline && (getuid()==0 || geteuid()==0 || getuid()!=geteuid())) throw std::runtime_error("run capture as an ordinary non-setuid user");
#ifndef __linux__
    if(!offline) throw std::runtime_error("live capture is Linux-only; --offline permits protocol tests without an SDK context");
#endif
    const std::uint16_t endian=1;
    if(*reinterpret_cast<const unsigned char*>(&endian)!=1) throw std::runtime_error("wire/Z16 requires a little-endian host");
    for(int fd:{STDIN_FILENO,STDOUT_FILENO}) {
        const int flags=fcntl(fd,F_GETFL);
        if(flags<0 || fcntl(fd,F_SETFL,flags|O_NONBLOCK)<0) throw std::runtime_error("cannot configure inherited pipes");
        if(isatty(fd)) throw std::runtime_error("binary protocol requires pipes, not a terminal");
    }
    std::signal(SIGINT,on_signal); std::signal(SIGTERM,on_signal); std::signal(SIGPIPE,SIG_IGN);
    std::uint32_t instance=0;
    while(!instance) if(getentropy(&instance,sizeof(instance))) throw std::runtime_error("cannot generate producer instance");
    std::array<std::atomic<std::uint64_t>,MAX_CAMERAS> counters{};
    std::unique_ptr<Capture> capture;
    bool owner=true; auto lease=raw_ns();
    auto release=[&]{ owner=false; if(capture) capture->stop(); };
    while(true) {
        if(capture && capture->deadline_expired()) {
            diagnostic("SDK operation exceeded deadline; exiting 124 without a hardware reset"); std::_Exit(124);
        }
        // Preserve terminal context-error status for the owner's diagnostics;
        // RELEASE/EOF still destroys an already-finished capture promptly.
        if(capture && capture->done() && capture->stopping()) capture.reset();
        if(owner && (signalled || raw_ns()-lease>10*SECOND)) release();
        if(!owner) {
            if(!capture) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(25)); continue;
        }
        pollfd pending{STDIN_FILENO,POLLIN,0};
        if(poll(&pending,1,50)<0) { if(errno==EINTR) continue; release(); continue; }
        if(!(pending.revents&POLLIN)) {
            if(pending.revents&(POLLHUP|POLLERR|POLLNVAL)) release();
            continue;
        }
        Request r{};
        if(!transfer(STDIN_FILENO,&r,sizeof(r),false,raw_ns()+2*SECOND)) { release(); continue; }
        if(!valid_request(r)) { respond(r.operation,Invalid,"{\"error\":\"invalid_request\"}"); release(); continue; }
        lease=raw_ns(); bool ok=true;
        if(r.operation==Hello) {
            ok=respond(Hello,Ok,"{\"protocol_version\":1,\"service\":\"rua-camera-capture-helper\",\"version\":\"1.0.0\",\"backend\":\"linux-stdio\",\"authorized_uid\":"+
                std::to_string(getuid())+",\"lease_s\":10,\"profile\":\"RGB8+Z16_640x480_30\",\"producer_instance\":"+quote(std::to_string(instance))+
                ",\"read_next\":true,\"queue_capacity\":8,\"offline\":"+(offline?"true":"false")+",\"compile_sdk_version\":"+std::to_string(RS2_API_VERSION)+"}");
        } else if(r.operation==Acquire) {
            if(offline) ok=respond(Acquire,NotReady,"{\"error\":\"offline_mode_no_usb\"}");
            else if(r.role_mask & ~((1u<<ROLES.size())-1u)) ok=respond(Acquire,Invalid,"{\"error\":\"camera_not_configured\"}");
            else if(capture) ok=respond(Acquire,Busy,"{\"error\":\"capture_already_active\"}");
            else { capture=std::make_unique<Capture>(instance,counters,r.role_mask); ok=respond(Acquire,Ok,capture->status()); }
        } else if(r.operation==Release) {
            if(capture) capture->stop();
            ok=respond(Release,Ok,"{\"state\":\"stopping\"}");
        } else if(r.operation==Read || r.operation==ReadNext) {
            if(!capture || capture->stopping() || !(capture->mask()&r.role_mask)) ok=respond(r.operation,NotReady,"{\"error\":\"role_not_acquired\"}");
            else {
                unsigned i=0; while((1u<<i)!=r.role_mask) ++i;
                std::shared_ptr<const Packet> packet; std::string status,metadata;
                bool has_latest=false;
                {
                    auto& slot=capture->slots[i]; std::lock_guard<std::mutex> lock(slot.mutex);
                    status=slot.status; has_latest=bool(slot.latest);
                    packet=r.operation==ReadNext?slot.history.next_after(r.after_sequence):slot.latest;
                    if(packet) {
                        const auto skipped=r.after_sequence && packet->sequence>r.after_sequence
                            ?packet->sequence-r.after_sequence-1:0;
                        std::ostringstream queue;
                        queue << ",\"capture_queue\":{\"selection\":" << quote(r.operation==ReadNext?"next_after":"latest")
                            << ",\"capacity\":" << FrameQueue::capacity << ",\"retained\":" << slot.history.size()
                            << ",\"overflow_dropped\":" << slot.history.overflow_dropped
                            << ",\"last_dropped_sequence\":" << quote(std::to_string(slot.history.last_dropped_sequence))
                            << ",\"requested_after_sequence\":" << quote(std::to_string(r.after_sequence))
                            << ",\"skipped_before_read\":" << skipped << "}}";
                        metadata=packet->metadata.substr(0,packet->metadata.size()-1)+queue.str();
                    }
                }
                const auto age=packet?raw_ns()-packet->arrival:-1;
                if(!packet && has_latest && r.operation==ReadNext) ok=respond(r.operation,NotNew,"{\"error\":\"not_new\"}");
                else if(!packet || age<0 || age>500000000) ok=respond(r.operation,NoFrame,"{\"error\":\"no_fresh_frame\",\"detail\":"+quote(status)+"}");
                else if(packet->sequence<=r.after_sequence) ok=respond(r.operation,NotNew,"{\"error\":\"not_new\"}");
                else ok=respond(r.operation,Ok,metadata,packet);
            }
        } else ok=respond(r.operation,Ok,capture?capture->status():"{\"state\":\"idle\"}");
        if(!ok) release();
    }
    return 0;
}
int self_test() {
    Request r{}; std::memcpy(r.magic,kRequestMagic,8); r.version=1; r.operation=Acquire; r.role_mask=7;
    if(!valid_request(r)) return 1;
    r.role_mask=1u<<MAX_CAMERAS; if(valid_request(r)) return 1;
    r.operation=Read; r.role_mask=3; if(valid_request(r)) return 1;
    r.role_mask=2; if(!valid_request(r)) return 1;
    r.operation=ReadNext; if(!valid_request(r)) return 1;
    r.role_mask=3; if(valid_request(r)) return 1;
    r.role_mask=2; r.reserved=1; if(valid_request(r)) return 1;
    auto packet=[](std::uint64_t seq) { auto p=std::make_shared<Packet>(); p->sequence=seq; return p; };
    FrameQueue queue;
    // A delayed reader receives B before C; preview/latest never consumes it.
    for(unsigned i=1;i<=6;++i) queue.push(packet(i));
    for(unsigned i=1;i<=6;++i) {
        const auto p=queue.next_after(i-1);
        if(!p || p->sequence!=i || queue.overflow_dropped) return 1;
    }
    if(queue.next_after(6)) return 1;
    queue.clear();
    for(unsigned i=1;i<=12;++i) queue.push(packet(i));
    if(queue.size()!=8 || queue.overflow_dropped!=4 || queue.last_dropped_sequence!=4 || queue.oldest()!=5 || queue.newest()!=12) return 1;
    if(queue.next_after(1)->sequence!=5 || queue.next_after(5)->sequence!=6) return 1;
    if(queue.next_after(5)->sequence!=6) return 1; // retried cursor is idempotent
    queue.clear(); queue.push(packet(13));
    if(queue.overflow_dropped || queue.next_after(0)->sequence!=13) return 1;
    if(quote("a\n\"\\")!="\"a\\u000a\\\"\\\\\"") return 1;
    if(raw_ns()<=0) return 1;
    std::cout << "{\"ok\":true,\"hardware_accessed\":false,\"request_bytes\":32,\"response_bytes\":32}\n";
    return 0;
}
} // namespace
int main(int argc,char** argv) {
    if(argc==2 && std::string(argv[1])=="--self-test") return self_test();
    if(argc==1 || (argc==2 && std::string(argv[1])=="--help")) {
        std::cout << "RUA ordinary-user Linux D405 RGB-D capture.\n"
                  << "--stdio: binary helper wire v1 on inherited stdin/stdout; no USB until ACQUIRE.\n"
                  << "--stdio --offline: same IPC, ACQUIRE refused, no SDK context or USB.\n"
                  << "--self-test: offline ABI checks. Diagnostics go only to stderr.\n";
        return 0;
    }
    const bool stdio=argc>=2 && std::string(argv[1])=="--stdio";
    bool offline=false;
    if(!stdio) return 2;
    try {
        for(int i=2;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--offline") offline=true;
            else if(arg.rfind("--camera=",0)==0) add_camera(arg.substr(9));
            else throw std::runtime_error("unknown capture argument");
        }
        if(!offline && ROLES.empty()) throw std::runtime_error("explicit --camera=role,ASIC,SDK configuration required");
        return serve(offline);
    } catch(const std::exception& error) { diagnostic(error.what()); return 1; }
}
