#include <array>
#include <functional>
#include <memory>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
namespace gazebo {
class CommunitySignal final : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model,sdf::ElementPtr sdf) override {
    model_=model;
    red_=sdf->Get<double>("redSeconds",10).first;
    green_=sdf->Get<double>("greenSeconds",15).first;
    yellow_=sdf->Get<double>("yellowSeconds",3).first;
    if (red_<=0 || green_<=0 || yellow_<=0) return;
    node_.reset(new transport::Node);node_->Init(model_->GetWorld()->Name());
    publisher_=node_->Advertise<msgs::Visual>("~/visual");
    if (ros::isInitialized()) {
      ros_node_.reset(new ros::NodeHandle);
      state_pub_=ros_node_->advertise<std_msgs::String>(
          "/simulation/traffic_lights/"+model_->GetName()+"/state",1,true);
    }
    connection_=event::Events::ConnectWorldUpdateBegin(std::bind(&CommunitySignal::Update,this));
  }
 private:
  void Update() {
    double now=model_->GetWorld()->SimTime().Double();
    double t=std::fmod(now,red_+green_+yellow_);
    int state=t<red_?0:(t<red_+green_?2:1);
    // Republish periodically so a newly connected Gazebo GUI receives the state.
    if (state==last_state_ && now>=last_ && now-last_<.5) return;
    last_=now;last_state_=state;
    const std::array<std::string,3> names={{"red","yellow","green"}};
    if (ros_node_) {
      std_msgs::String state_msg;state_msg.data=names[state];state_pub_.publish(state_msg);
    }
    for (int i=0;i<3;++i) {
      msgs::Visual msg;auto parent=model_->GetLink("body")->GetScopedName();
      msg.set_name(parent+"::"+names[i]);msg.set_parent_name(parent);
      auto material=msg.mutable_material();
      auto script=material->mutable_script();
      script->set_name("Community/signal_"+names[i]+(i==state?"_on":"_off"));
      script->add_uri("model://smart_community_sim/materials/scripts");
      script->add_uri("model://smart_community_sim/materials/textures");
      material->set_lighting(false);
      publisher_->Publish(msg);
    }
  }
  physics::ModelPtr model_;
  transport::NodePtr node_;
  transport::PublisherPtr publisher_;
  event::ConnectionPtr connection_;
  std::unique_ptr<ros::NodeHandle> ros_node_;
  ros::Publisher state_pub_;
  double red_=10,green_=15,yellow_=3,last_=-1;
  int last_state_=-1;
};
GZ_REGISTER_MODEL_PLUGIN(CommunitySignal)
}
