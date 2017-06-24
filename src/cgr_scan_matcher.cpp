//
// Created by kandithws on 24/6/2560.
//

#include "cgr_scan_matcher.h"

namespace cgr_slam {

  CgrScanMatcher::CgrScanMatcher() {

  }

  double CgrScanMatcher::computeIcpLinearStep(Pose2D &pret,
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
    std::list<GMapping::PointPair> pairs;

    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
      skip++;
      skip=skip>m_likelihoodSkip?0:skip;
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
      for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
        for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
          GMapping::IntPoint pr=iphit+GMapping::IntPoint(xx,yy);
          GMapping::IntPoint pf=pr+ipfree;
          //AccessibilityState s=map.storage().cellState(pr);
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

    Pose2D result(0,0,0);
    // double icpError=icpNonlinearStep(result,pairs);
    double icpError=GMapping::icpStep(result,pairs);
    std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
    pret.x=p.x+result.x;
    pret.y=p.y+result.y;
    pret.theta=p.theta+result.theta;
    pret.theta=atan2(sin(pret.theta), cos(pret.theta));
    // return score(map, p, readings);
    return icpError;
  }

  double CgrScanMatcher::computeIcpNonLinearStep(Pose2D &pret,
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
    std::list<GMapping::PointPair> pairs;

    for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
      skip++;
      skip=skip>m_likelihoodSkip?0:skip;
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
      for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
        for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
          GMapping::IntPoint pr=iphit+GMapping::IntPoint(xx,yy);
          GMapping::IntPoint pf=pr+ipfree;
          //AccessibilityState s=map.storage().cellState(pr);
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

    Pose2D result(0,0,0);
    double icpError=GMapping::icpNonlinearStep(result,pairs);

    std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
    pret.x=p.x+result.x;
    pret.y=p.y+result.y;
    pret.theta=p.theta+result.theta;
    pret.theta=atan2(sin(pret.theta), cos(pret.theta));
    // return score(map, p, readings);
    return icpError;
  }

} //END namespace cgr_slam