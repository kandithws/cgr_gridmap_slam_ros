//
// Created by kandithws on 24/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H
#define CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H

#include <gmapping/scanmatcher/smmap.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/scanmatcher/icp.h>
#include "utils/param_macros.h"

namespace cgr_slam{
  class CgrScanMatcher : public GMapping::ScanMatcher {
   public:
    typedef GMapping::OrientedPoint Pose2D;
    CgrScanMatcher();
    double computeIcpLinearStep(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                const Pose2D& p, const double* readings);
    double computeIcpNonLinearStep(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                   const Pose2D& p, const double* readings);

    protected:
     void matchGreedyBeamEndPoint(std::list<GMapping::PointPair>& pairs,
                                  const GMapping::ScanMatcherMap &map,
                                  const Pose2D& p, const double *readings);

     void matchBeamRayTrace(std::list<GMapping::PointPair>& pairs,
                            const GMapping::ScanMatcherMap &map,
                            const Pose2D& p, const double *readings);

     private:
      double calcMapRayHitRange(const GMapping::ScanMatcherMap &map, const Pose2D& laser_pose,
                                const double& angle); // Return Meters to hit map cell

    CGR_PARAM(ScanMatchKernelSize, sm_kern_size, int, 1)
    CGR_PARAM(ScanMatchBeamSkip, sm_beam_skip, int, 0)
    CGR_PARAM(UseRayTrace, use_ray_trace, bool, false)
  };
}



#endif //CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H
