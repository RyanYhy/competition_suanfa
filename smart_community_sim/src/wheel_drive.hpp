#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace community_drive {
inline double Clamp(double value,double bound) {
  return std::max(-bound,std::min(bound,value));
}
inline double AngleDelta(double current,double previous) {
  return std::atan2(std::sin(current-previous),std::cos(current-previous));
}
struct Motion { double x=0,y=0,yaw=0; };
inline Motion WheelMotion(const std::array<double,4>& value,double radius,double lever) {
  return {radius*(value[0]+value[1]+value[2]+value[3])/4,
          radius*(-value[0]+value[1]+value[2]-value[3])/4,
          radius*(-value[0]+value[1]-value[2]+value[3])/(4*lever)};
}
inline void Integrate(Motion& pose,const Motion& delta) {
  const double middle=pose.yaw+delta.yaw/2;
  pose.x+=std::cos(middle)*delta.x-std::sin(middle)*delta.y;
  pose.y+=std::sin(middle)*delta.x+std::cos(middle)*delta.y;
  pose.yaw=AngleDelta(pose.yaw+delta.yaw,0);
}
// Encoder counts come from measured hinge angle changes, never commanded speed.
class Encoders {
 public:
  void Reset(const std::array<double,4>& position) { previous_=position; }
  std::array<double,4> Update(const std::array<double,4>& position) {
    std::array<double,4> delta{};
    for(size_t i=0;i<4;++i){delta[i]=AngleDelta(position[i],previous_[i]);previous_[i]=position[i];}
    return delta;
  }
 private: std::array<double,4> previous_{};
};
// Bounded torque servo with low-pass encoder speed and conditional anti-windup.
// Integral torque permits holding against a constant load at zero target speed.
class Motor {
 public:
  void Reset() { integral_=0;speed_=0; }
  double Update(double target,double speed,double dt,double kp,double ki,double limit,double tau) {
    speed_+=(speed-speed_)*dt/(tau+dt);
    const double error=target-speed_;
    const double candidate=Clamp(integral_+ki*error*dt,limit);
    const double proposed=kp*error+candidate;
    if(std::abs(proposed)<=limit || proposed*error<0)integral_=candidate;
    if(ki==0)integral_=0;
    return Clamp(kp*error+integral_,limit);
  }
  double Speed() const { return speed_; }
 private: double integral_=0,speed_=0;
};
}
