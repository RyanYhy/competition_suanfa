#ifndef MY_LOCAL_PLANNER_H_
#define MY_LOCAL_PLANNER_H_
#include <nav_core/base_local_planner.h>
#include <base_local_planner/costmap_model.h>
#include <memory>
namespace my_local_planner {
// Adaptation of yhy's lookahead/error-feedback planner for a holonomic chassis.
class My_Local_Planner : public nav_core::BaseLocalPlanner {
 public:
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd) override;
  bool isGoalReached() override { return reached_; }
 private:
  bool legal(const geometry_msgs::PoseStamped& pose, const geometry_msgs::Twist& cmd);
  tf2_ros::Buffer* tf_ = nullptr;
  costmap_2d::Costmap2DROS* costmap_ = nullptr;
  std::unique_ptr<base_local_planner::CostmapModel> model_;
  std::vector<geometry_msgs::PoseStamped> plan_;
  geometry_msgs::PoseStamped goal_, previous_pose_;
  ros::Time previous_time_, stopped_since_;
  bool reached_ = false, have_goal_ = false, aligning_goal_ = false;
  double held_yaw_ = 0, measured_w_ = 0;
  double max_v_ = .18, max_w_ = .5, min_w_ = .16;
  double xy_tol_ = .018, yaw_tol_ = .04, lookahead_ = .16;
};
}
#endif
