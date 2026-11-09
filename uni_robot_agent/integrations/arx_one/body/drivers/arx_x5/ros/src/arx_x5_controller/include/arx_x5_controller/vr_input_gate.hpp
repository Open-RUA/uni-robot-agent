#pragma once

#include <chrono>
#include <cstdint>

namespace arx::x5 {

class VrInputGate {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

  struct Decision {
    bool active = false;
    bool use_vr = false;
    bool new_frame = false;
  };

  double takeover_sec = 0.1;
  double hold_sec = 0.3;
  double input_timeout_sec = 0.3;
  double release_cooldown_sec = 0.5;

  bool Fresh(Time received_at, Time now) const {
    const double age = std::chrono::duration<double>(now - received_at).count();
    return age >= 0.0 && age < input_timeout_sec;
  }

  Decision Update(std::uint64_t sequence, Time received_at, bool moving,
                  bool using_vr, Time now) {
    Decision decision;
    decision.new_frame = sequence != 0 && sequence != last_sequence_;
    last_sequence_ = sequence;
    if (sequence == 0 || !Fresh(received_at, now)) {
      above_tracking_ = false;
      return decision;
    }

    bool takeover_ready = false;
    if (decision.new_frame) {
      const bool cooling_down = released_ &&
          std::chrono::duration<double>(received_at - released_at_).count() < release_cooldown_sec;
      decision.active = moving && !cooling_down;
      if (decision.active) {
        // Receipt gaps break takeover tracking, even if the timer was delayed.
        if (above_tracking_ && !Fresh(last_active_at_, received_at)) {
          above_tracking_ = false;
        }
        last_active_at_ = received_at;
        if (!using_vr) {
          if (!above_tracking_) {
            above_tracking_ = true;
            above_since_ = received_at;
          } else {
            takeover_ready =
                std::chrono::duration<double>(received_at - above_since_).count() >= takeover_sec;
          }
        }
      } else {
        above_tracking_ = false;
      }
    }

    if (using_vr) {
      above_tracking_ = false;
      decision.use_vr = decision.active ||
          std::chrono::duration<double>(now - last_active_at_).count() < hold_sec;
    } else {
      decision.use_vr = takeover_ready;
    }
    return decision;
  }

  void Release(Time now) {
    above_tracking_ = false;
    released_ = true;
    released_at_ = now;
  }

 private:
  std::uint64_t last_sequence_ = 0;
  bool above_tracking_ = false;
  bool released_ = false;
  Time above_since_{};
  Time last_active_at_{};
  Time released_at_{};
};

}  // namespace arx::x5
