//
// Created by kandithws on 6/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_SLAM_ROS_WRAPPER_H
#define CGR_GRIDMAP_SLAM_ROS_SLAM_ROS_WRAPPER_H

#include <iostream>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/Float64.h>
#include <message_filters/subscriber.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include "cgr_slam2d_processor.h"

#define ENABLE_MAP_RENDERER_THREAD

class SlamRosWrapper{
 public:
  SlamRosWrapper();
  ~SlamRosWrapper();
  void executeNode();
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  bool initNodeFromFirstScan(const sensor_msgs::LaserScan& scan);
  void mapRendererThread();

 private:
  bool isRosShutdown(){return !ros::ok();}
  void initProcessorInstance();
  bool initProcessorParams();
  void broadcastMapTransform();
  bool processCgrSlam(const sensor_msgs::LaserScan& scan, cgr_slam::Pose2D& odom_pose);
  bool getOdomToRobotLaser(cgr_slam::Pose2D& odom_laser_pose, ros::Time time);
  void updateMap();
  double computePoseEntropy();

  boost::shared_ptr<cgr_slam::CgrSlam2DProcessor> slam_proc_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher entropy_publisher_;
  ros::Publisher sst_;
  ros::Publisher sstm_;
  ros::AsyncSpinner spinner_;
  tf::TransformListener tf_;
  boost::mutex tf_map_to_odom_mutex_;
  tf::Transform tf_map_to_odom_;
  boost::mutex map_mutex_;
  boost::mutex wait_first_scan_mutex_;
  boost::mutex slam_mutex_;
  boost::mutex render_map_mutex_;
  boost::condition_variable wait_render_map_signal_;
  boost::condition_variable wait_first_scan_signal_;
  boost::shared_ptr<boost::thread> map_renderer_thread_;
  ros::Time last_map_update_;
  tf::Stamped<tf::Pose> centered_laser_pose_;
  bool is_first_scan_received_ = false;
  ros::Subscriber laser_sub_;
  tf::TransformBroadcaster tf_bc_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> > scan_filter_sub_;
  boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan> > scan_filter_;
  nav_msgs::GetMap::Response map_;

  // ROS Node Params
  double tf_publish_period_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_frame_;
  std::string laser_frame_;
  double map_render_interval_;
  bool do_reverse_range_;
  // The angles in the laser, going from -x to x (adjustment is made to get the laser between
  // symmetrical bounds as that's what gmapping expects)
  std::vector<double> laser_angles_;
  double laser_max_range_;
  double laser_min_range_;
  cgr_slam::LaserScanSensor* laser_sensor_;
  //boost::shared_ptr<cgr_slam::OdomSensor> odom_sensor_;
  // Map parameters
  bool got_map_ = false;
  double x_min_size_;
  double x_max_size_;
  double y_min_size_;
  double y_max_size_;
  double map_resol_;
  double occ_thresh_;
  double resample_th_;
  bool force_render_ = false;
  bool use_gmapping_;


  // CGR Params
  double laser_max_usable_range_;
  double linear_u_dist_;
  double angular_u_dist_;
  int particles_;
  int laser_total_beam_count_;

};

#endif //CGR_GRIDMAP_SLAM_ROS_SLAM_ROS_WRAPPER_H
