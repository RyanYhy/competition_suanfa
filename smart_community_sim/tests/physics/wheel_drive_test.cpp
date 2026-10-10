#include "../../src/wheel_drive.hpp"
#include <cassert>
#include <iostream>
using namespace community_drive;
int main() {
  Encoders encoder;encoder.Reset({{3.14,0,0,0}});
  auto delta=encoder.Update({{-3.14,0,0,0}});
  assert(std::abs(delta[0]-(2*M_PI-6.28))<1e-12);
  encoder.Reset({{1,2,3,4}});
  Motion stationary;
  // A solver velocity bias cannot move encoder odometry without angle changes.
  for(int i=0;i<10000;++i)Integrate(stationary,WheelMotion(encoder.Update({{1,2,3,4}}),.054,.27));
  assert(stationary.x==0 && stationary.y==0 && stationary.yaw==0);
  auto turn=WheelMotion({{-5,5,-5,5}},.054,.27);
  assert(std::abs(turn.yaw-1)<1e-12 && turn.x==0 && turn.y==0);
  Motor motor;double speed=0;
  // Physical inertia plus a constant disturbing load. No direct velocity setting.
  for(int i=0;i<10000;++i){double torque=motor.Update(0,speed,.001,.04,.08,.45,.01);assert(std::abs(torque)<=.45);speed+=(torque-.02-.005*speed)/.002*.001;}
  assert(std::abs(speed)<1e-5);
  motor.Reset();
  for(int i=0;i<10000;++i)assert(motor.Update(100,0,.001,.04,.08,.45,.01)==.45);
  // Anti-windup: a saturated stalled request must not leave forward torque at stop.
  assert(motor.Update(0,0,.001,.04,.08,.45,.01)==0);
  motor.Reset();assert(motor.Speed()==0);
  std::cout<<"encoder wrap, stationary counts, mecanum yaw, torque bounds, zero-speed load rejection and anti-windup passed\n";
}
