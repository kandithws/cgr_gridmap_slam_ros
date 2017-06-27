//
// Created by kandithws on 20/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_MATH_UTILS_H
#define CGR_GRIDMAP_SLAM_ROS_MATH_UTILS_H

#include "math.h"

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}

static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

#endif //CGR_GRIDMAP_SLAM_ROS_MATH_UTILS_H
