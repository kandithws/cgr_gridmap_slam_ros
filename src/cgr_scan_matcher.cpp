//
// Created by kandithws on 24/6/2560.
//

#include <limits>
#include <utils/print_utils_macros.h>
#include "cgr_scan_matcher.h"

namespace cgr_slam {

  CgrScanMatcher::CgrScanMatcher() {

  }

  double CgrScanMatcher::computeIcpLinearStep(Pose2D &pret,
                                              const GMapping::ScanMatcherMap &map,
                                              const Pose2D &p,
                                              const double *readings) {
    std::list<GMapping::PointPair> pairs;
    if (use_ray_trace_) {
      matchBeamRayTrace(pairs, map, p, readings);
    }
    else{
      matchGreedyBeamEndPoint(pairs, map, p, readings);
    }


    Pose2D result(0,0,0);
    // double icpError=icpNonlinearStep(result,pairs);
    double icpError=GMapping::icpStep(result,pairs);
    //std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;

    if (isnan(result.x) || isnan(result.y)) {
      icpError = std::numeric_limits<double>::infinity();
      LOGPRINT_DEBUG("Linear-ICP Diverge, point pairs=%ld", pairs.size());
    }
    else{
      pret.x=p.x+result.x;
      pret.y=p.y+result.y;
      pret.theta=p.theta+result.theta;
      pret.theta=atan2(sin(pret.theta), cos(pret.theta));
    }

    // return score(map, p, readings);

    return icpError;
  }

  double CgrScanMatcher::computeIcpNonLinearStep(Pose2D &pret,
                                                 const GMapping::ScanMatcherMap &map,
                                                 const Pose2D &p,
                                                 const double *readings) {
    std::list<GMapping::PointPair> pairs;
    if (use_ray_trace_) {
      matchBeamRayTrace(pairs, map, p, readings);
    }
    else{
      matchGreedyBeamEndPoint(pairs, map, p, readings);
    }

    Pose2D result(0,0,0);
    double icpError=GMapping::icpNonlinearStep(result,pairs);

    //std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
    if (isnan(result.x) || isnan(result.y)) {
      icpError = std::numeric_limits<double>::infinity();
      LOGPRINT_DEBUG("Non Linear-ICP Diverge, point pairs=%ld", pairs.size());
    }
    else{
      pret.x=p.x+result.x;
      pret.y=p.y+result.y;
      pret.theta=p.theta+result.theta;
      pret.theta=atan2(sin(pret.theta), cos(pret.theta));
    }
    // return score(map, p, readings);
    return icpError;
  }


  void CgrScanMatcher::matchGreedyBeamEndPoint(std::list<GMapping::PointPair> &pairs,
                                               const GMapping::ScanMatcherMap &map,
                                               const Pose2D &p,
                                               const double *readings) {

    const double * angle=m_laserAngles+m_initialBeamsSkip;
    Pose2D lp=p;
    lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
    lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
    lp.theta+=m_laserPose.theta;
    unsigned int skip=0;
    double freeDelta=map.getDelta()*m_freeCellRatio;
    //std::list<GMapping::PointPair> pairs;

    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
      skip++;
      //skip=skip>m_likelihoodSkip?0:skip;
      skip=skip>sm_beam_skip_?0:skip;
      if (*r>m_usableRange||*r==0.0) continue;
      if (skip) continue;
      GMapping::Point phit=lp;
      phit.x+=*r*cos(lp.theta+*angle);
      phit.y+=*r*sin(lp.theta+*angle);
      GMapping::IntPoint iphit=map.world2map(phit);
      GMapping::Point pfree=lp;
      pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
      pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
      pfree=pfree-phit;
      GMapping::IntPoint ipfree=map.world2map(pfree);
      bool found=false;
      GMapping::Point bestMu(0.,0.);
      GMapping::Point bestCell(0.,0.);
      //for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
      for (int xx=-sm_kern_size_; xx<=sm_kern_size_; xx++)
        //for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
        for (int yy=-sm_kern_size_; yy<=sm_kern_size_; yy++){
          GMapping::IntPoint pr=iphit+GMapping::IntPoint(xx,yy);
          GMapping::IntPoint pf=pr+ipfree;
          //map.storage().isInside(pr);
          //GMapping::AccessibilityState s=map.storage().cellState(pr);
          //if (s&Inside && s&Allocated){
          const GMapping::PointAccumulator& cell=map.cell(pr);
          const GMapping::PointAccumulator& fcell=map.cell(pf);
          if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
            GMapping::Point mu=phit-cell.mean();
            if (!found){
              bestMu=mu;
              bestCell=cell.mean();
              found=true;
            }else
            if((mu*mu)<(bestMu*bestMu)){
              bestMu=mu;
              bestCell=cell.mean();
            }

          }
          //}
        }
      if (found){
        pairs.push_back(std::make_pair(phit, bestCell));
        //std::cerr << "(" << phit.x-bestCell.x << "," << phit.y-bestCell.y << ") ";
      }
      //std::cerr << std::endl;
    }

  }

  void CgrScanMatcher::matchBeamRayTrace(std::list<GMapping::PointPair> &pairs,
                                         const GMapping::ScanMatcherMap &map,
                                         const Pose2D &p,
                                         const double *readings) {
    // TODO -- TEST
    //LOGPRINT_WARN("Ray Tracing, total skip: %d", sm_beam_skip_);
    const double * angle=m_laserAngles+m_initialBeamsSkip;
    Pose2D lp=p;
    lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
    lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
    lp.theta+=m_laserPose.theta;
    unsigned int skip=0;
    // double freeDelta=map.getDelta()*m_freeCellRatio;
    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++) {
      skip++;
      //skip=skip>m_likelihoodSkip?0:skip;
      skip = skip > sm_beam_skip_ ? 0 : skip;
      if (*r > m_usableRange || *r == 0.0) continue;
      if (skip) continue;
      double mu = calcMapRayHitRange(map, lp, *angle);
      if(!isinf(mu)){
        GMapping::Point phit=lp;
        phit.x+=*r*cos(lp.theta+*angle);
        phit.y+=*r*sin(lp.theta+*angle);
        GMapping::Point bestCell=lp;
        bestCell.x+=mu*cos(lp.theta+*angle);
        bestCell.y+=mu*sin(lp.theta+*angle);
        pairs.push_back(std::make_pair(phit, bestCell));
      }
      else{
        // Not Match
      }
    }
  }

  double CgrScanMatcher::calcMapRayHitRange(const GMapping::ScanMatcherMap &map,
                                          const Pose2D &laser_pose,
                                          const double &angle) {
    // Bresenham raytracing
    int x0,x1,y0,y1;
    int x,y;
    int xstep, ystep;
    char steep;
    int tmp;
    int deltax, deltay, error, deltaerr;

    // World to Map

    //x0 = MAP_GXWX(map,ox);
    //y0 = MAP_GYWY(map,oy);
    GMapping::IntPoint p0 = map.world2map(laser_pose);
    x0 = p0.x;
    y0 = p0.y;
    //x1 = MAP_GXWX(map,ox + max_range * cos(oa));
    //y1 = MAP_GYWY(map,oy + max_range * sin(oa));
    Pose2D lp_max = laser_pose;
    lp_max.x+=m_laserMaxRange*cos(laser_pose.theta+angle);
    lp_max.y+=m_laserMaxRange*sin(laser_pose.theta+angle);
    GMapping::IntPoint p1 = map.world2map(lp_max);
    x1 = p1.x;
    y1 = p1.y;

    if(abs(y1-y0) > abs(x1-x0))
      steep = 1;
    else
      steep = 0;

    if(steep)
    {
      tmp = x0;
      x0 = y0;
      y0 = tmp;

      tmp = x1;
      x1 = y1;
      y1 = tmp;
    }

    deltax = abs(x1-x0);
    deltay = abs(y1-y0);
    error = 0;
    deltaerr = deltay;

    x = x0;
    y = y0;

    if(x0 < x1)
      xstep = 1;
    else
      xstep = -1;
    if(y0 < y1)
      ystep = 1;
    else
      ystep = -1;

    if(steep)
    {
      //if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
      //  return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map->scale;
      // TODO -- Change Occupancy Threshold to parameter
      const GMapping::PointAccumulator& cell=map.cell(y,x);
      // Map->scale (m/cell)
      if(map.storage().isInside(y,x) || ((double)cell )> 0.5)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.getDelta();
    }
    else
    {
      //if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
      const GMapping::PointAccumulator& cell=map.cell(x,y);
      if(map.storage().isInside(x,y) || ((double)cell )> 0.5)
        return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.getDelta();
    }

    while(x != (x1 + xstep * 1))
    {
      x += xstep;
      error += deltaerr;
      if(2*error >= deltax)
      {
        y += ystep;
        error -= deltax;
      }

      if(steep)
      {
        //if(!MAP_VALID(map,y,x) || map->cells[MAP_INDEX(map,y,x)].occ_state > -1)
        const GMapping::PointAccumulator& cell=map.cell(y,x);
        // Map->scale (m/cell)
        if(map.storage().isInside(y,x) || ((double)cell )> 0.5)
          return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.getDelta();
      }
      else
      {
        //if(!MAP_VALID(map,x,y) || map->cells[MAP_INDEX(map,x,y)].occ_state > -1)
        const GMapping::PointAccumulator& cell=map.cell(x,y);
        if(map.storage().isInside(x,y) || ((double)cell )> 0.5)
          return sqrt((x-x0)*(x-x0) + (y-y0)*(y-y0)) * map.getDelta();
      }
    }
    //return m_laserMaxRange;
    return std::numeric_limits<double>::infinity();
  }

}//END namespace cgr_slam