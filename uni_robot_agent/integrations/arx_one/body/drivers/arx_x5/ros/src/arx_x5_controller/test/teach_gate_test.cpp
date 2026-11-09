#include "arx_x5_controller/teach_gate.hpp"
#include <cassert>
#include <limits>
struct Status { double position; };
struct Fake {
  std::vector<int> errors, writes;
  std::vector<double> q{.1,.2,.3,.4,.5,.6,-6.};
  std::vector<double> v = std::vector<double>(7,0.);
  double jaw=-.4, target_jaw=99.;
  std::vector<double> target;
  auto getErrorCode(){return errors;}
  auto getJointPositons(){return q;}
  auto getJointVelocities(){return v;}
  auto getJointStatus(){return std::vector<Status>(7,Status{jaw});}
  void setArmStatus(int mode){writes.push_back(mode);}
  void setJointPositions(std::vector<double> value){target=value; writes.push_back(10);}
  void setCatch(double value){target_jaw=value; writes.push_back(11);}
};
template<class F> void fails(F f){bool caught=false;try{f();}catch(const std::exception&){caught=true;}assert(caught);}
int main(){
  arx::x5::TeachGate gate; Fake driver;
  fails([&]{gate.resume(driver,2,100);});assert(driver.writes.empty());
  gate.enter(driver);assert(gate.active()&&!gate.accepts(200));assert(driver.writes==std::vector<int>{0});
  fails([&]{gate.enter(driver);});
  driver.q[1]=.7;driver.jaw=-.2; // Human has dragged arm AND gripper.
  driver.v[3]=.1;
  fails([&]{gate.resume(driver,2,100);});assert(gate.active()&&driver.writes.size()==1);
  driver.v[3]=std::numeric_limits<double>::quiet_NaN();
  fails([&]{gate.resume(driver,2,100);});assert(gate.active());
  driver.v[3]=0;driver.errors={47};
  fails([&]{gate.resume(driver,2,100);});assert(gate.active()&&driver.writes.size()==1);
  driver.errors.clear();gate.resume(driver,2,100);
  assert(!gate.active()&&gate.accepts(101)&&!gate.accepts(99)&&!gate.accepts(0));
  assert(driver.target[1]==.7&&driver.target_jaw==-.2);
  assert(driver.writes==std::vector<int>({0,10,11,5}));
  gate.enter(driver);assert(!gate.accepts(102));
}
