//
// Created by kandithws on 7/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_MOTION_MODEL_H
#define CGR_GRIDMAP_SLAM_ROS_MOTION_MODEL_H

#include <map>
#include <gmapping/utils/point.h>
#include <boost/shared_ptr.hpp>
#include "print_utils_macros.h"
#include "math_utils.h"



namespace cgr_slam{
class MotionModel{
 public:
  typedef GMapping::OrientedPoint Pose2D;
  MotionModel();
  void setParams(double srr, double srt, double str, double stt);
  GMapping::OrientedPoint drawFromMotion();
  // Implement sample_motion_odometry (Prob Rob p 136) -- Diff drive
  GMapping::OrientedPoint drawFromMotionTrueModel(const Pose2D& p, const Pose2D& pnew, const Pose2D& pold);
  /* Gmapping: Gaussian approximation of Odometry Motion Model (Diff Drive) through Taylor series expansion*/
  GMapping::OrientedPoint drawFromMotionEKFLinearized(const Pose2D& p, const Pose2D& pnew, const Pose2D& pold);
  static double normalizeAngle(double z);
  static double diffAngle(double a, double b) {
   double d1, d2;
   a = normalizeAngle(a);
   b = normalizeAngle(b);
   d1 = a-b;
   d2 = 2*M_PI - fabs(d1);
   if(d1 > 0)
    d2 *= -1.0;
   if(fabs(d1) < fabs(d2))
    return(d1);
   else
    return(d2);
  }
 private:
  double srr_;
  double srt_;
  double str_;
  double stt_;
};
}


#endif //CGR_GRIDMAP_SLAM_ROS_MOTION_MODEL_H
