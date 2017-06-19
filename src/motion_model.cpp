//
// Created by kandithws on 7/6/2560.
//
#include "motion_model.h"
#include <gmapping/utils/stat.h>


cgr_slam::MotionModel::MotionModel(){

}

void cgr_slam::MotionModel::setParams(double srr, double srt, double str, double stt) {
  srr_ =srr;
  srt_ = srt;
  str_= str;
  stt_ = stt;
}


GMapping::OrientedPoint cgr_slam::MotionModel::drawFromMotion() {

  return GMapping::OrientedPoint(0,0,0);
}

GMapping::OrientedPoint cgr_slam::MotionModel::drawFromMotionEKFLinearized(const cgr_slam::Pose2D& p,
                                                    const cgr_slam::Pose2D& pnew, const cgr_slam::Pose2D& pold){
  double sxy=0.3*srr_;
  Pose2D delta=absoluteDifference(pnew, pold);
  Pose2D noisypoint(delta);
  noisypoint.x+=GMapping::sampleGaussian(srr_*fabs(delta.x)+str_*fabs(delta.theta)+sxy*fabs(delta.y));
  noisypoint.y+=GMapping::sampleGaussian(srr_*fabs(delta.y)+str_*fabs(delta.theta)+sxy*fabs(delta.x));
  noisypoint.theta+=GMapping::sampleGaussian(stt_*fabs(delta.theta)+srt_*sqrt(delta.x*delta.x+delta.y*delta.y));
  noisypoint.theta=fmod(noisypoint.theta, 2*M_PI);
  if (noisypoint.theta>M_PI)
    noisypoint.theta-=2*M_PI;
  return absoluteSum(p,noisypoint);
}



