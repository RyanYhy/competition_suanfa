// Adapted from jie_ware's gradient-map / neighboring-pose search localizer.
// Cartesian laser extrinsics, bounded coarse/fine search and scan-time odometry.
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <vector>
#include <algorithm>
class LidarLoc {
 public:
  LidarLoc(): nh_(), private_("~"), listener_(buffer_) {
    private_.param<std::string>("base_frame",base_,"base_footprint");
    private_.param<std::string>("odom_frame",odom_,"odom");
    private_.param<std::string>("laser_topic",scan_topic_,"/scan");
    private_.param("initial_pose_x",initial_x_,0.);
    private_.param("initial_pose_y",initial_y_,0.);
    private_.param("initial_pose_a",initial_yaw_,0.);
    private_.param("minimum_score",minimum_score_,.55);
    private_.param("transform_tolerance",tolerance_,.15);
    map_sub_=nh_.subscribe("map",1,&LidarLoc::map,this);
    scan_sub_=nh_.subscribe(scan_topic_,1,&LidarLoc::scan,this);
    initial_sub_=nh_.subscribe("initialpose",1,&LidarLoc::initial,this);
    pose_pub_=nh_.advertise<geometry_msgs::PoseStamped>("lidar_pose",1);
    score_pub_=nh_.advertise<std_msgs::Float32>("lidar_match_score",1);
    timer_=nh_.createTimer(ros::Duration(.02),&LidarLoc::broadcast,this);
  }
 private:
  void map(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (msg->info.resolution<=0 || msg->data.size()!=size_t(msg->info.width)*msg->info.height) return;
    if (std::abs(tf2::getYaw(msg->info.origin.orientation))>1e-6) {
      ROS_ERROR("lidar_loc requires an axis-aligned occupancy map"); return;
    }
    cv::Mat free(msg->info.height,msg->info.width,CV_8UC1,cv::Scalar(255));
    int occupied=0;
    for(unsigned y=0;y<msg->info.height;++y) for(unsigned x=0;x<msg->info.width;++x) {
      if(msg->data[y*msg->info.width+x]>=65) {free.at<unsigned char>(y,x)=0; ++occupied;}
    }
    if (!occupied) {ROS_ERROR("lidar_loc: empty occupied map"); return;}
    metadata_=msg->info;
    cv::distanceTransform(free,distances_,cv::DIST_L2,cv::DIST_MASK_PRECISE);
    distances_*=metadata_.resolution;
    ROS_INFO("lidar_loc: map ready; headless scan matching, no ground-truth input");
  }
  void initial(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (msg->header.frame_id!="map") {ROS_WARN("initialpose must be in map"); return;}
    initial_x_=msg->pose.pose.position.x;initial_y_=msg->pose.pose.position.y;
    initial_yaw_=tf2::getYaw(msg->pose.pose.orientation);
    seeded_=false;valid_=false;
  }
  double score(double x,double y,double yaw,const std::vector<cv::Point2d>& points) const {
    double sum=0;const double c=std::cos(yaw),s=std::sin(yaw);
    for(const auto& p:points) {
      const double mx=(x+c*p.x-s*p.y-metadata_.origin.position.x)/metadata_.resolution-.5;
      const double my=(y+s*p.x+c*p.y-metadata_.origin.position.y)/metadata_.resolution-.5;
      const int ix=std::floor(mx),iy=std::floor(my);
      if(ix<0 || iy<0 || ix+1>=distances_.cols || iy+1>=distances_.rows) continue;
      const double fx=mx-ix,fy=my-iy;
      const double d=(1-fy)*((1-fx)*distances_.at<float>(iy,ix)+fx*distances_.at<float>(iy,ix+1))+
                           fy*((1-fx)*distances_.at<float>(iy+1,ix)+fx*distances_.at<float>(iy+1,ix+1));
      sum+=std::max(0.,1.-d/.15);
    }
    return sum/points.size();
  }
  void scan(const sensor_msgs::LaserScan::ConstPtr& msg) {
    if(distances_.empty()) return;
    geometry_msgs::TransformStamped laser_msg,odom_msg;
    try {
      laser_msg=buffer_.lookupTransform(base_,msg->header.frame_id,msg->header.stamp,ros::Duration(.08));
      odom_msg=buffer_.lookupTransform(odom_,base_,msg->header.stamp,ros::Duration(.08));
    } catch(const tf2::TransformException& ex) {ROS_WARN_THROTTLE(2,"lidar_loc TF: %s",ex.what());return;}
    tf2::Transform laser,odom_pose;tf2::fromMsg(laser_msg.transform,laser);tf2::fromMsg(odom_msg.transform,odom_pose);
    std::vector<cv::Point2d> points;
    const size_t stride=std::max<size_t>(1,msg->ranges.size()/180);
    for(size_t i=0;i<msg->ranges.size();i+=stride) {
      const double range=msg->ranges[i];
      if(!std::isfinite(range)||range<msg->range_min||range>=msg->range_max) continue;
      const double a=msg->angle_min+i*msg->angle_increment;
      auto p=laser*tf2::Vector3(range*std::cos(a),range*std::sin(a),0);
      points.emplace_back(p.x(),p.y());
    }
    if(points.size()<30) return;
    tf2::Transform predicted;
    if(seeded_) predicted=map_to_odom_*odom_pose;
    else {
      predicted.setOrigin(tf2::Vector3(initial_x_,initial_y_,0));
      tf2::Quaternion q;q.setRPY(0,0,initial_yaw_);predicted.setRotation(q);
    }
    double x=predicted.getOrigin().x(),y=predicted.getOrigin().y(),yaw=tf2::getYaw(predicted.getRotation());
    const double px=x,py=y,pa=yaw;
    double best=score(x,y,yaw,points);
    // Original neighbor search (5 translations x 3 angles), now bounded and refined.
    for(const auto step: std::vector<std::pair<double,double>>{{.02,M_PI/180.},{.005,M_PI/720.}}) {
      for(int iteration=0;iteration<(seeded_?30:80);++iteration) {
        double bx=x,by=y,ba=yaw,bscore=best;
        for(int axis=0;axis<5;++axis) for(int turn=-1;turn<=1;++turn) {
          const double nx=x+(axis==1?step.first:axis==2?-step.first:0);
          const double ny=y+(axis==3?step.first:axis==4?-step.first:0);
          const double na=yaw+turn*step.second;
          if(seeded_ && (std::hypot(nx-px,ny-py)>.08 || std::abs(na-pa)>.12)) continue;
          const double candidate=score(nx,ny,na,points);
          if(candidate>bscore+1e-6) {bscore=candidate;bx=nx;by=ny;ba=na;}
        }
        if(bscore<=best+1e-6) break;
        x=bx;y=by;yaw=ba;best=bscore;
      }
    }
    std_msgs::Float32 quality;quality.data=best;score_pub_.publish(quality);
    if(best<minimum_score_) {ROS_WARN_THROTTLE(1,"lidar_loc rejected match: %.3f",best);return;}
    tf2::Transform map_base;map_base.setOrigin(tf2::Vector3(x,y,0));
    tf2::Quaternion q;q.setRPY(0,0,yaw);map_base.setRotation(q);
    // Both factors refer to this scan timestamp. Broadcast the correction, not an old body pose.
    map_to_odom_=map_base*odom_pose.inverse();seeded_=true;valid_=true;last_scan_=msg->header.stamp;
    geometry_msgs::PoseStamped pose;pose.header=msg->header;pose.header.frame_id="map";
    pose.pose.position.x=x;pose.pose.position.y=y;pose.pose.orientation=tf2::toMsg(q);pose_pub_.publish(pose);
  }
  void broadcast(const ros::TimerEvent&) {
    const auto now=ros::Time::now();
    if(!valid_ || (now-last_scan_).toSec()>.5 || now<last_scan_ || now==last_broadcast_) return;
    last_broadcast_=now;
    geometry_msgs::TransformStamped tf;tf.header.stamp=now+ros::Duration(tolerance_);
    tf.header.frame_id="map";tf.child_frame_id=odom_;tf.transform=tf2::toMsg(map_to_odom_);broadcaster_.sendTransform(tf);
  }
  ros::NodeHandle nh_,private_;tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
  tf2_ros::TransformBroadcaster broadcaster_;ros::Subscriber map_sub_,scan_sub_,initial_sub_;
  ros::Publisher pose_pub_,score_pub_;ros::Timer timer_;std::string base_,odom_,scan_topic_;
  cv::Mat distances_;nav_msgs::MapMetaData metadata_;tf2::Transform map_to_odom_;
  double initial_x_,initial_y_,initial_yaw_,minimum_score_,tolerance_;
  bool seeded_=false,valid_=false;ros::Time last_scan_,last_broadcast_;
};
int main(int argc,char** argv) {ros::init(argc,argv,"lidar_loc");LidarLoc localizer;ros::spin();return 0;}
