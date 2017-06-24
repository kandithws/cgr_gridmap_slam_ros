//
// Created by kandithws on 24/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H
#define CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H

#include <gmapping/scanmatcher/smmap.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/scanmatcher/icp.h>


namespace cgr_slam{
  class CgrScanMatcher : public GMapping::ScanMatcher {
   public:
    typedef GMapping::OrientedPoint Pose2D;
    CgrScanMatcher();
    double computeIcpLinearStep(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                const Pose2D& p, const double* readings);
    double computeIcpNonLinearStep(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                   const Pose2D& p, const double* readings);

  };
}



#endif //CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H
