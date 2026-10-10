// Standalone ODE contact diagnostic. Not a Gazebo/ROS integration test.
#include <ode/ode.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "../../src/wheel_drive.hpp"
struct Joint { dJointID id; double damping; };
struct Simulation {
  dWorldID world=dWorldCreate(); dSpaceID space=dHashSpaceCreate(0);
  dJointGroupID contacts=dJointGroupCreate(0); dGeomID floor;
  std::map<std::string,dBodyID> bodies; std::map<std::string,Joint> joints;
  double kp=100000, kd=10;
  Simulation() {
    dWorldSetGravity(world,0,0,-9.81);dWorldSetQuickStepNumIterations(world,100);
    dWorldSetQuickStepW(world,1);dWorldSetERP(world,.2);dWorldSetCFM(world,0);
    dWorldSetContactMaxCorrectingVel(world,.1);dWorldSetContactSurfaceLayer(world,.0001);
    floor=dCreatePlane(space,0,0,1,0);
  }
  static void collide(void* context,dGeomID a,dGeomID b) {
    auto& s=*static_cast<Simulation*>(context);
    if(a!=s.floor && b!=s.floor)return; // URDF selfCollide=false.
    dContact c[4]{};
    const int count=dCollide(a,b,4,&c[0].geom,sizeof(dContact));
    for(int i=0;i<count;++i){
      c[i].surface.mode=dContactApprox1|dContactSoftERP|dContactSoftCFM;
      c[i].surface.mu=1.;
      c[i].surface.soft_erp=.001*s.kp/(.001*s.kp+s.kd);
      c[i].surface.soft_cfm=1./(.001*s.kp+s.kd);
      auto joint=dJointCreateContact(s.world,s.contacts,&c[i]);
      dJointAttach(joint,dGeomGetBody(a),dGeomGetBody(b));
    }
  }
  void load(const char* path) {
    std::ifstream in(path);std::string kind,name;
    while(in>>kind>>name){
      if(kind=="body"){
        double mass,i[6],p[3];dQuaternion q;
        in>>mass;for(auto& v:i)in>>v;for(auto& v:p)in>>v;for(auto& v:q)in>>v;
        auto b=dBodyCreate(world);bodies[name]=b;dMass m;
        dMassSetParameters(&m,mass,0,0,0,i[0],i[1],i[2],i[3],i[4],i[5]);
        dBodySetMass(b,&m);dBodySetPosition(b,p[0],p[1],p[2]);dBodySetQuaternion(b,q);
      }else if(kind=="shape"){
        std::string type;double dims[3],p[3];dQuaternion q;
        in>>type;for(auto& v:dims)in>>v;for(auto& v:p)in>>v;for(auto& v:q)in>>v;
        dGeomID geom=type=="sphere"?dCreateSphere(space,dims[0]):
          type=="cylinder"?dCreateCylinder(space,dims[0],dims[1]):dCreateBox(space,dims[0],dims[1],dims[2]);
        dGeomSetBody(geom,bodies.at(name));dGeomSetOffsetPosition(geom,p[0],p[1],p[2]);dGeomSetOffsetQuaternion(geom,q);
      }else if(kind=="joint"){
        std::string parent,child,type;double p[3],axis[3],damping;
        in>>parent>>child>>type;for(auto& v:p)in>>v;for(auto& v:axis)in>>v;in>>damping;
        dJointID joint=type=="fixed"?dJointCreateFixed(world,0):dJointCreateHinge(world,0);
        dJointAttach(joint,bodies.at(child),bodies.at(parent));
        if(type=="fixed")dJointSetFixed(joint);
        else {dJointSetHingeAnchor(joint,p[0],p[1],p[2]);dJointSetHingeAxis(joint,axis[0],axis[1],axis[2]);joints[name]={joint,damping};}
      }else throw std::runtime_error("Unknown input record");
    }
  }
};
double yaw(dBodyID b){const auto r=dBodyGetRotation(b);return std::atan2(r[4],r[0]);}
int main(int argc,char**argv){
  if(argc<6)return 2;
  dInitODE2(0);Simulation s;s.load(argv[1]);
  const double vx=std::stod(argv[2]),vy=std::stod(argv[3]),wz=std::stod(argv[4]);
  const double gain=argc>6?std::stod(argv[6]):.04;
  const bool revised=argc>7 && std::string(argv[7])=="encoder_pi";
  const std::array<std::string,4> names={{"front_left","front_right","rear_left","rear_right"}};
  std::array<dJointID,4>wheels;for(int i=0;i<4;++i)wheels[i]=s.joints.at(names[i]+"_wheel_joint").id;
  std::ofstream out(argv[5]);out<<"time,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw,encoder_wz,true_wz\n";
  double x=0,y=0,a=0;std::array<double,4>target{};
  std::array<double,4> previous{};
  std::array<community_drive::Motor,4> motors;
  for(int i=0;i<4;++i)previous[i]=dJointGetHingeAngle(wheels[i]);
  for(int step=0;step<17000;++step){
    const bool active=step>=2000 && step<6000;
    const double turn=.27*wz;
    const std::array<double,4> desired=active?std::array<double,4>{{(vx-vy-turn)/.054,(vx+vy+turn)/.054,(vx+vy-turn)/.054,(vx-vy+turn)/.054}}:std::array<double,4>{{0,0,0,0}};
    for(const auto& entry:s.joints)dJointAddHingeTorque(entry.second.id,-entry.second.damping*dJointGetHingeAngleRate(entry.second.id));
    std::array<double,4>speed;
    for(int i=0;i<4;++i){
      target[i]+=std::max(-.03,std::min(.03,desired[i]-target[i]));
      const double position=dJointGetHingeAngle(wheels[i]);
      const double encoder_speed=community_drive::AngleDelta(position,previous[i])/.001;
      previous[i]=position;
      speed[i]=revised?encoder_speed:dJointGetHingeAngleRate(wheels[i]);
      const double torque=revised?motors[i].Update(target[i],speed[i],.001,gain,.08,.45,.01):
          std::max(-.45,std::min(.45,gain*(target[i]-speed[i])));
      dJointAddHingeTorque(wheels[i],torque);
    }
    const double bx=.054*(speed[0]+speed[1]+speed[2]+speed[3])/4;
    const double by=.054*(-speed[0]+speed[1]+speed[2]-speed[3])/4;
    const double omega=.054*(-speed[0]+speed[1]-speed[2]+speed[3])/(4*.27);
    x+=(std::cos(a+omega*.0005)*bx-std::sin(a+omega*.0005)*by)*.001;
    y+=(std::sin(a+omega*.0005)*bx+std::cos(a+omega*.0005)*by)*.001;a+=omega*.001;
    dSpaceCollide(s.space,&s,Simulation::collide);dWorldQuickStep(s.world,.001);dJointGroupEmpty(s.contacts);
    if(step%20==0){auto body=s.bodies.at("base_link");const auto p=dBodyGetPosition(body);
      out<<step*.001<<','<<p[0]<<','<<p[1]<<','<<yaw(body)<<','<<x<<','<<y<<','<<a<<','<<omega<<','<<dBodyGetAngularVel(body)[2]<<'\n';}
  }
  dJointGroupDestroy(s.contacts);dSpaceDestroy(s.space);dWorldDestroy(s.world);dCloseODE();
}
