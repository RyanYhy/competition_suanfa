#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ODESurfaceParams.hh>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "wheel_drive.hpp"

namespace gazebo {
// The only actuator is joint torque. Ground contact supplies all chassis motion.
class MecanumDrivePlugin final : public ModelPlugin {
 public:
  ~MecanumDrivePlugin() override { connection_.reset(); sub_.shutdown(); queue_.disable(); }
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    if (!ros::isInitialized()) { gzerr << "Load through gazebo_ros.\n"; return; }
    const std::array<std::string,4> names = {{"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint"}};
    for (size_t i=0; i<4; ++i) {
      wheels_[i]=model_->GetJoint(names[i]);
      if (!wheels_[i]) { gzerr << "Missing " << names[i] << "\n"; return; }
    }
    radius_=sdf->Get<double>("wheelRadius",0.054).first;
    lever_=sdf->Get<double>("wheelX",0.13).first+sdf->Get<double>("wheelY",0.14).first;
    timeout_=sdf->Get<double>("commandTimeout",0.5).first;
    kp_=sdf->Get<double>("motorKp",0.04).first;
    ki_=sdf->Get<double>("motorKi",0.08).first;
    velocity_filter_=sdf->Get<double>("motorVelocityFilter",0.01).first;
    max_torque_=sdf->Get<double>("maxTorque",0.45).first;
    max_speed_=sdf->Get<double>("maxWheelSpeed",8).first;
    initial_x_=sdf->Get<double>("initialX",0).first;
    initial_y_=sdf->Get<double>("initialY",0).first;
    initial_yaw_=sdf->Get<double>("initialYaw",0).first;
    if (!(radius_>0 && lever_>0 && timeout_>0 && kp_>0 && ki_>=0 && velocity_filter_>=0 && max_torque_>0 && max_speed_>0)) {
      gzerr << "Invalid drive parameters\n"; return;
    }
    // sdformat9 drops link-level contact extensions on named multi-collisions.
    // Apply them to every roller shape after URDF conversion as well.
    for (const auto& link:model_->GetLinks()) {
      if (link->GetName().find("_roller_")==std::string::npos) continue;
      for (const auto& collision:link->GetCollisions()) {
        auto surface=boost::dynamic_pointer_cast<physics::ODESurfaceParams>(collision->GetSurface());
        if (!surface) continue;
        surface->FrictionPyramid()->SetMuPrimary(1.0);
        surface->FrictionPyramid()->SetMuSecondary(1.0);
        surface->kp=100000; surface->kd=10;
        collision->SetMaxContacts(4);
      }
    }
    node_.reset(new ros::NodeHandle);
    node_->setCallbackQueue(&queue_);
    sub_=node_->subscribe("cmd_vel",1,&MecanumDrivePlugin::Command,this);
    odom_pub_=node_->advertise<nav_msgs::Odometry>("odom",10);
    joint_pub_=node_->advertise<sensor_msgs::JointState>("joint_states",10);
    // Evaluation-only output. Never used to correct encoder odometry or publish TF.
    truth_pub_=node_->advertise<nav_msgs::Odometry>("/simulation/ground_truth",10);
    broadcaster_.reset(new tf::TransformBroadcaster);
    ROS_INFO_STREAM("Mecanum drive: encoder-angle odometry, bounded PI torque servo; kp=" << kp_
                    << " ki=" << ki_ << " velocity_filter=" << velocity_filter_);
    Reset();
    connection_=event::Events::ConnectWorldUpdateBegin(std::bind(&MecanumDrivePlugin::Update,this));
  }
  void Reset() override {
    cmd_=geometry_msgs::Twist(); targets_.fill(0);
    accumulated_.fill(0);truth_initialized_=false;
    std::array<double,4> position{};
    for(size_t i=0;i<4;++i){if(wheels_[i])position[i]=wheels_[i]->Position(0);motors_[i].Reset();}
    encoders_.Reset(position);
    mismatch_since_=-1;
    x_=initial_x_; y_=initial_y_; yaw_=initial_yaw_;
    if (model_) last_=last_cmd_=last_pub_=model_->GetWorld()->SimTime().Double();
  }
 private:
  void Command(const geometry_msgs::Twist::ConstPtr& msg) {
    if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->linear.y)||!std::isfinite(msg->angular.z)) return;
    cmd_=*msg; last_cmd_=model_->GetWorld()->SimTime().Double();
  }
  static double Clamp(double x,double limit) { return std::max(-limit,std::min(limit,x)); }
  void Update() {
    const double now=model_->GetWorld()->SimTime().Double();
    if (now<last_) Reset();
    queue_.callAvailable(ros::WallDuration(0));
    const double dt=now-last_; last_=now;
    if (dt<=0) return;
    const bool fresh=now-last_cmd_<=timeout_;
    const double vx=fresh?cmd_.linear.x:0, vy=fresh?cmd_.linear.y:0, turn=fresh?lever_*cmd_.angular.z:0;
    std::array<double,4> desired={{(vx-vy-turn)/radius_,(vx+vy+turn)/radius_,(vx+vy-turn)/radius_,(vx-vy+turn)/radius_}};
    double peak=max_speed_;
    for (double v:desired) peak=std::max(peak,std::abs(v));
    std::array<double,4> speed,position,raw_speed;
    for(size_t i=0;i<4;++i){position[i]=wheels_[i]->Position(0);raw_speed[i]=wheels_[i]->GetVelocity(0);}
    const auto delta=encoders_.Update(position);
    for (size_t i=0;i<4;++i) {
      desired[i]*=max_speed_/peak;
      targets_[i]+=Clamp(desired[i]-targets_[i],30.0*dt);
      speed[i]=delta[i]/dt;
      accumulated_[i]+=delta[i];
      wheels_[i]->SetForce(0,motors_[i].Update(targets_[i],speed[i],dt,kp_,ki_,max_torque_,velocity_filter_));
    }
    community_drive::Motion integrated{x_,y_,yaw_};
    community_drive::Integrate(integrated,community_drive::WheelMotion(delta,radius_,lever_));
    x_=integrated.x;y_=integrated.y;yaw_=integrated.yaw;
    if (now-last_pub_<0.02) return;
    const double publish_dt=now-last_pub_;
    const auto travelled=community_drive::WheelMotion(accumulated_,radius_,lever_);
    const double bx=travelled.x/publish_dt,by=travelled.y/publish_dt,wz=travelled.yaw/publish_dt;
    accumulated_.fill(0);
    last_pub_=now;
    ros::Time stamp; stamp.fromSec(now);
    const auto q=tf::createQuaternionFromYaw(yaw_);
    nav_msgs::Odometry odom;
    odom.header.stamp=stamp; odom.header.frame_id="odom"; odom.child_frame_id="base_footprint";
    odom.pose.pose.position.x=x_; odom.pose.pose.position.y=y_;
    tf::quaternionTFToMsg(q,odom.pose.pose.orientation);
    odom.twist.twist.linear.x=bx; odom.twist.twist.linear.y=by; odom.twist.twist.angular.z=wz;
    for (int i=0;i<6;++i) {
      odom.pose.covariance[i*7]=(i==0||i==1||i==5)?0.02:1e6;
      odom.twist.covariance[i*7]=(i==0||i==1||i==5)?0.04:1e6;
    }
    odom_pub_.publish(odom);
    nav_msgs::Odometry truth;
    truth.header.stamp=stamp;truth.header.frame_id="world";truth.child_frame_id="base_footprint";
    const auto pose=model_->WorldPose();
    truth.pose.pose.position.x=pose.Pos().X();truth.pose.pose.position.y=pose.Pos().Y();truth.pose.pose.position.z=pose.Pos().Z();
    truth.pose.pose.orientation.x=pose.Rot().X();truth.pose.pose.orientation.y=pose.Rot().Y();
    truth.pose.pose.orientation.z=pose.Rot().Z();truth.pose.pose.orientation.w=pose.Rot().W();
    // Report actual pose changes over the same interval as encoder increments.
    // Solver instantaneous velocity remains diagnostic only, not a displacement sensor.
    const auto linear=truth_initialized_?pose.Rot().RotateVectorReverse((pose.Pos()-previous_truth_pose_.Pos())/publish_dt):ignition::math::Vector3d::Zero;
    truth.twist.twist.linear.x=linear.X();truth.twist.twist.linear.y=linear.Y();
    truth.twist.twist.angular.z=truth_initialized_?community_drive::AngleDelta(pose.Rot().Yaw(),previous_truth_pose_.Rot().Yaw())/publish_dt:0;
    previous_truth_pose_=pose;truth_initialized_=true;
    truth_pub_.publish(truth);
    // Read-only diagnostics: these values never affect motor torque or odometry.
    ROS_DEBUG_STREAM_THROTTLE(2.0,"drive_check t=" << now
      << " cmd_vxyw=[" << vx << "," << vy << "," << (fresh?cmd_.angular.z:0) << "]"
      << " encoder_vxyw=[" << bx << "," << by << "," << wz << "]"
      << " actual_vxyw=[" << linear.X() << "," << linear.Y() << "," << truth.twist.twist.angular.z << "]"
      << " odom_xyyaw=[" << x_ << "," << y_ << "," << yaw_ << "]"
      << " actual_xyyaw=[" << pose.Pos().X() << "," << pose.Pos().Y() << "," << pose.Rot().Yaw() << "]"
      << " wheels=[" << speed[0] << "," << speed[1] << "," << speed[2] << "," << speed[3] << "]"
      << " solver_wheels=[" << raw_speed[0] << "," << raw_speed[1] << "," << raw_speed[2] << "," << raw_speed[3] << "]"
      << " wheel_angles=[" << position[0] << "," << position[1] << "," << position[2] << "," << position[3] << "]"
      << " solver_actual_wz=" << model_->WorldAngularVel().Z());
    if (std::abs(wz-truth.twist.twist.angular.z)>.03 ||
        std::hypot(bx-linear.X(),by-linear.Y())>.02) {
      if (mismatch_since_<0) mismatch_since_=now;
      if (now-mismatch_since_>1.0)
        ROS_WARN_THROTTLE(2.0,"DRIVE_ODOMETRY_MISMATCH: wheel motion differs from body motion; inspect contacts/slip before localization tuning");
    } else mismatch_since_=-1;
    broadcaster_->sendTransform(tf::StampedTransform(tf::Transform(q,tf::Vector3(x_,y_,0)),stamp,"odom","base_footprint"));
    sensor_msgs::JointState states; states.header.stamp=stamp;
    for (const auto& joint:model_->GetJoints()) if (joint->DOF()>0) {
      states.name.push_back(joint->GetName()); states.position.push_back(joint->Position(0)); states.velocity.push_back(joint->GetVelocity(0));
    }
    joint_pub_.publish(states);
  }
  physics::ModelPtr model_;
  std::array<physics::JointPtr,4> wheels_;
  std::array<double,4> targets_{};
  std::array<double,4> accumulated_{};
  community_drive::Encoders encoders_;
  std::array<community_drive::Motor,4> motors_;
  ignition::math::Pose3d previous_truth_pose_;
  bool truth_initialized_=false;
  std::unique_ptr<ros::NodeHandle> node_;
  std::unique_ptr<tf::TransformBroadcaster> broadcaster_;
  ros::CallbackQueue queue_;
  ros::Subscriber sub_;
  ros::Publisher odom_pub_,joint_pub_,truth_pub_;
  event::ConnectionPtr connection_;
  geometry_msgs::Twist cmd_;
  double radius_=0.054,lever_=0.27,timeout_=0.5,kp_=0.04,max_torque_=0.45,max_speed_=8;
  double ki_=0.08,velocity_filter_=0.01;
  double last_=0,last_cmd_=0,last_pub_=0,x_=0,y_=0,yaw_=0;
  double initial_x_=0,initial_y_=0,initial_yaw_=0;
  double mismatch_since_=-1;
};
GZ_REGISTER_MODEL_PLUGIN(MecanumDrivePlugin)
}
