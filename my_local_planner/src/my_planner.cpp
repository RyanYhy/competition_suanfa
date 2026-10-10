#include "my_local_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <angles/angles.h>
#include <algorithm>
#include <cmath>
PLUGINLIB_EXPORT_CLASS(my_local_planner::My_Local_Planner, nav_core::BaseLocalPlanner)
namespace my_local_planner {
void My_Local_Planner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap) {
  if (costmap_) return;
  tf_ = tf; costmap_ = costmap;
  model_.reset(new base_local_planner::CostmapModel(*costmap_->getCostmap()));
  ros::NodeHandle nh("~/" + name);
  nh.param("max_vel_trans", max_v_, max_v_);
  nh.param("max_vel_theta", max_w_, max_w_);
  nh.param("min_turn_velocity", min_w_, min_w_);
  nh.param("xy_goal_tolerance", xy_tol_, xy_tol_);
  nh.param("yaw_goal_tolerance", yaw_tol_, yaw_tol_);
  nh.param("lookahead_distance", lookahead_, lookahead_);
  ROS_INFO("yhy lookahead planner: holonomic, footprint checked, visualization disabled");
}
bool My_Local_Planner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!costmap_ || plan.empty()) return false;
  const auto& end = plan.back();
  const bool changed = !have_goal_ || end.header.frame_id != goal_.header.frame_id ||
      std::hypot(end.pose.position.x-goal_.pose.position.x,end.pose.position.y-goal_.pose.position.y) > .005 ||
      std::abs(angles::shortest_angular_distance(tf2::getYaw(goal_.pose.orientation),tf2::getYaw(end.pose.orientation))) > .005;
  if (changed) {
    geometry_msgs::PoseStamped pose;
    if (!costmap_->getRobotPose(pose)) return false;
    held_yaw_ = tf2::getYaw(pose.pose.orientation);
    reached_ = false; aligning_goal_ = false; stopped_since_ = ros::Time();
    goal_ = end; have_goal_ = true;
  }
  plan_ = plan;
  return true;
}
bool My_Local_Planner::legal(const geometry_msgs::PoseStamped& pose,const geometry_msgs::Twist& cmd) {
  double x=pose.pose.position.x, y=pose.pose.position.y, yaw=tf2::getYaw(pose.pose.orientation);
  const auto footprint = costmap_->getRobotFootprint();
  auto* grid = costmap_->getCostmap();
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*grid->getMutex());
  // Swept footprint with a one-second horizon (greater than the nominal braking time).
  for (int i=0; i<=25; ++i) {
    unsigned mx,my;
    if (!grid->worldToMap(x,y,mx,my) || grid->getCost(mx,my)>=253 || model_->footprintCost(x,y,yaw,footprint,0,0)<0) return false;
    const double dt=.04;
    x += (cmd.linear.x*std::cos(yaw)-cmd.linear.y*std::sin(yaw))*dt;
    y += (cmd.linear.x*std::sin(yaw)+cmd.linear.y*std::cos(yaw))*dt;
    yaw += cmd.angular.z*dt;
  }
  return true;
}
bool My_Local_Planner::computeVelocityCommands(geometry_msgs::Twist& cmd) {
  cmd = geometry_msgs::Twist();
  if (!costmap_ || plan_.empty() || !costmap_->isCurrent()) return false;
  geometry_msgs::PoseStamped pose, end;
  if (!costmap_->getRobotPose(pose)) return false;
  std::vector<geometry_msgs::PoseStamped> local;
  try {
    for (auto point: plan_) {
      point.header.stamp = ros::Time(0);
      local.push_back(tf_->transform(point,costmap_->getGlobalFrameID(),ros::Duration(.05)));
    }
    auto point=goal_; point.header.stamp=ros::Time(0);
    end=tf_->transform(point,costmap_->getGlobalFrameID(),ros::Duration(.05));
  } catch (const tf2::TransformException& ex) {
    ROS_WARN_THROTTLE(1,"Planner TF: %s",ex.what()); return false;
  }
  const auto now=ros::Time::now();
  const double yaw=tf2::getYaw(pose.pose.orientation), dt=(now-previous_time_).toSec();
  if (dt>.02 && dt<.5) {
    const double rate=angles::shortest_angular_distance(tf2::getYaw(previous_pose_.pose.orientation),yaw)/dt;
    measured_w_ = .5*rate+.5*measured_w_;
  }
  previous_time_=now; previous_pose_=pose;
  const double dx=end.pose.position.x-pose.pose.position.x, dy=end.pose.position.y-pose.pose.position.y;
  const double distance=std::hypot(dx,dy);
  const double final_error=angles::shortest_angular_distance(yaw,tf2::getYaw(end.pose.orientation));
  if (distance<=xy_tol_ && std::abs(final_error)<=yaw_tol_) {
    if (std::abs(measured_w_)<.06) {
      if (stopped_since_.isZero()) stopped_since_=now;
      reached_=(now-stopped_since_).toSec()>=.3;
    } else stopped_since_=ros::Time();
    return legal(pose,cmd);
  }
  stopped_since_=ros::Time();
  // Enter rotation only after centering. Small translation during the turn must
  // not switch back to the travelling heading and undo the observation turn.
  if (distance<=xy_tol_) aligning_goal_=true;
  const double heading_error=aligning_goal_ ? final_error : angles::shortest_angular_distance(yaw,held_yaw_);
  if (std::abs(heading_error)>yaw_tol_) {
    const double magnitude=std::min(max_w_,std::max(min_w_,1.8*std::abs(heading_error)));
    cmd.angular.z=std::copysign(magnitude,heading_error);
  }
  if (distance>xy_tol_ && std::abs(heading_error)<.10) {
    size_t nearest=0; double closest=1e9;
    for (size_t i=0;i<local.size();++i) {
      const double d=std::hypot(local[i].pose.position.x-pose.pose.position.x,local[i].pose.position.y-pose.pose.position.y);
      if (d<closest) {closest=d; nearest=i;}
    }
    size_t target=nearest; double arc=0;
    while (target+1<local.size() && arc<lookahead_) {
      arc+=std::hypot(local[target+1].pose.position.x-local[target].pose.position.x,local[target+1].pose.position.y-local[target].pose.position.y);
      ++target;
    }
    double tx=local[target].pose.position.x-pose.pose.position.x, ty=local[target].pose.position.y-pose.pose.position.y;
    if (distance<.12 || aligning_goal_) {tx=dx; ty=dy;}
    const double norm=std::hypot(tx,ty);
    if (norm>1e-6) {
      const double speed=std::min(aligning_goal_ ? .06 : max_v_,std::max(.045,1.2*norm));
      cmd.linear.x=speed*(tx*std::cos(yaw)+ty*std::sin(yaw))/norm;
      cmd.linear.y=speed*(-tx*std::sin(yaw)+ty*std::cos(yaw))/norm;
    }
  }
  for (double scale: {1., .5, .25}) {
    auto candidate=cmd;
    candidate.linear.x*=scale; candidate.linear.y*=scale; candidate.angular.z*=scale;
    if (legal(pose,candidate)) {cmd=candidate; return true;}
  }
  cmd=geometry_msgs::Twist();
  ROS_WARN_THROTTLE(1,"yhy planner: swept footprint blocked");
  return false;
}
}
