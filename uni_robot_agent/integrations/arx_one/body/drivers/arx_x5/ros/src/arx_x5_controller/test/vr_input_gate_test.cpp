#include "arx_x5_controller/vr_input_gate.hpp"

#include <iostream>
#include <stdexcept>

using arx::x5::VrInputGate;

VrInputGate::Time at(int milliseconds) {
  return VrInputGate::Time{} + std::chrono::milliseconds(milliseconds);
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

void take_over(VrInputGate& gate) {
  require(!gate.Update(1, at(0), false, false, at(0)).use_vr, "first frame is not movement");
  require(!gate.Update(2, at(10), true, false, at(10)).use_vr, "takeover needs sustained receipts");
  require(gate.Update(3, at(110), true, false, at(110)).use_vr, "fresh movement takes over");
}

int main() {
  {
    VrInputGate gate;
    require(!gate.Update(0, at(0), true, false, at(0)).use_vr, "no input cannot take over");
    gate.Update(1, at(0), false, false, at(0));
    gate.Update(2, at(10), true, false, at(10));
    // The original bug counted these same two cached frames on every tick.
    for (int tick = 11; tick != 1000; ++tick) {
      const auto decision = gate.Update(2, at(10), true, false, at(tick));
      require(!decision.active && !decision.use_vr && !decision.new_frame,
              "cached movement cannot advance takeover");
    }
  }
  {
    VrInputGate gate;
    take_over(gate);
    for (int tick = 111; tick != 410; ++tick) {
      const auto decision = gate.Update(3, at(110), true, true, at(tick));
      require(!decision.active && decision.use_vr && !decision.new_frame,
              "hold ownership without replaying pose or gripper commands");
    }
    require(!gate.Update(3, at(110), true, true, at(410)).use_vr,
            "stopped stream releases at the deadline");
  }
  {
    VrInputGate gate;
    gate.hold_sec = 10.0;
    take_over(gate);
    require(!gate.Update(3, at(110), true, true, at(410)).use_vr,
            "deadman overrides a long movement hold");
    require(!gate.Update(4, at(120), true, true, at(500)).use_vr,
            "a newly observed but expired frame cannot command");
  }
  {
    VrInputGate gate;
    gate.hold_sec = 0.15;
    take_over(gate);
    const auto moving = gate.Update(4, at(120), true, true, at(120));
    require(moving.active && moving.use_vr && moving.new_frame,
            "new moving input remains commandable");
    for (int tick = 130; tick <= 260; tick += 10) {
      const auto still = gate.Update(tick, at(tick), false, true, at(tick));
      require(!still.active && still.use_vr && still.new_frame, "fresh still frames preserve hold");
    }
    require(!gate.Update(270, at(270), false, true, at(270)).use_vr,
            "still receipts cannot refresh movement hold");
  }
  {
    VrInputGate gate;
    gate.Update(1, at(0), true, false, at(0));
    gate.Update(2, at(50), false, false, at(50));
    require(!gate.Update(3, at(110), true, false, at(110)).use_vr,
            "stationary input resets continuous takeover tracking");
    require(gate.Update(4, at(210), true, false, at(210)).use_vr,
            "takeover can restart after stationary input");
  }
  {
    VrInputGate gate;
    gate.Update(1, at(0), true, false, at(0));
    // Simulate a timer that missed the entire outage and receives a fresh pair.
    require(!gate.Update(4, at(400), true, false, at(400)).use_vr,
            "receipt gap cannot count toward takeover");
    require(gate.Update(5, at(500), true, false, at(500)).use_vr,
            "fresh sustained input after gap can take over");
    require(!gate.Fresh(at(0), at(300)), "callback must discard pre-outage comparison frame");
    require(gate.Fresh(at(300), at(310)), "callback can compare consecutive fresh receipts");
  }
  {
    VrInputGate gate;
    take_over(gate);
    gate.Release(at(410));
    require(!gate.Update(0, at(0), false, false, at(411)).use_vr, "release clears ownership");
    require(!gate.Update(4, at(500), true, false, at(500)).active, "release cooldown blocks movement");
    require(!gate.Update(5, at(900), true, false, at(950)).active,
            "delayed cooldown receipt cannot become active when processed later");
    require(!gate.Update(5, at(900), true, false, at(960)).use_vr,
            "cached cooldown receipt cannot retrigger takeover");
    require(!gate.Update(6, at(960), true, false, at(960)).use_vr,
            "first post-cooldown movement starts a new window");
    require(gate.Update(7, at(1060), true, false, at(1060)).use_vr,
            "fresh post-cooldown movement can retake control");
  }
  {
    VrInputGate gate;
    require(!gate.Update(1, at(100), true, false, at(99)).use_vr, "future timestamp is rejected");
    gate.Update(2, at(200), true, false, at(250));
    require(!gate.Update(3, at(250), true, false, at(400)).use_vr,
            "timer delay does not count as received movement duration");
    require(gate.Update(4, at(310), true, false, at(410)).use_vr,
            "takeover duration is measured between receipts");
  }
  std::cout << "VR receipt freshness regression passed\n";
}
