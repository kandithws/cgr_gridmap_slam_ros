//
// Created by kandithws on 24/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H
#define CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H

#include <gmapping/scanmatcher/smmap.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/scanmatcher/icp.h>
#include "utils/param_macros.h"

#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/slam/CICP.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPosePDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#define CGR_SIMPLE_GRADIENT 0
#define CGR_ICP_LINEAR 1
#define CGR_ICP_NONLINEAR 2
#define CGR_ICP_MRPT 3


namespace cgr_slam{
  class CgrScanMatcher : public GMapping::ScanMatcher {
   public:
    typedef GMapping::OrientedPoint Pose2D;
    CgrScanMatcher();
    double computeIcpLinearStep(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                const Pose2D& p, const double* readings);
    double computeIcpNonLinearStep(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                   const Pose2D& p, const double* readings);

    double computeGeneralizedICP(Pose2D & pret, const GMapping::ScanMatcherMap& map,
                                 const Pose2D& p, const double* readings);

    protected:
     void matchGreedyBeamEndPoint(std::list<GMapping::PointPair>& pairs,
                                  const GMapping::ScanMatcherMap &map,
                                  const Pose2D& p, const double *readings);

     void matchBeamRayTrace(std::list<GMapping::PointPair>& pairs,
                            const GMapping::ScanMatcherMap &map,
                            const Pose2D& p, const double *readings);

     void matchGreedyBeamEndPoint(mrpt::maps::CSimplePointsMap& particle_map,
                                  mrpt::maps::CSimplePointsMap& reading_map,
                                  const GMapping::ScanMatcherMap &map,
                                  const Pose2D& p, const double *readings);

     void matchBeamRayTrace(mrpt::maps::CSimplePointsMap& particle_map,
                             mrpt::maps::CSimplePointsMap& reading_map,
                             const GMapping::ScanMatcherMap &map,
                             const Pose2D& p, const double *readings);

     private:
      double calcMapRayHitRange(const GMapping::ScanMatcherMap &map, const Pose2D& laser_pose,
                                const double& angle); // Return Meters to hit map cell
      template <typename PointPairContainer>
      double icpStep(GMapping::OrientedPoint & retval, const PointPairContainer& container);

      template <typename PointPairContainer>
      double icpNonlinearStep(GMapping::OrientedPoint & retval, const PointPairContainer& container);

    CGR_PARAM(ScanMatchKernelSize, sm_kern_size, int, 1)
    CGR_PARAM(ScanMatchBeamSkip, sm_beam_skip, int, 0)
    CGR_PARAM(UseRayTrace, use_ray_trace, bool, false)
    CGR_PARAM(MaxAllowDistance, max_allow_distance, double, 1.0)

  };
}

#include "impl/icp.hpp"

#endif //CGR_GRIDMAP_SLAM_ROS_CGR_SCAN_MATCHER_H
