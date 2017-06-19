//
// Created by kandithws on 6/6/2560.
//

#include <iostream>
#include <ros/ros.h>
#include "slam_ros_wrapper.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "cgr_slam");
  SlamRosWrapper node;
  ROS_INFO("**Start Node: CGR SLAM**");
  node.executeNode();
  return 0;
}