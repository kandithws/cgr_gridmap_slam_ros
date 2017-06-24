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


GMapping::OrientedPoint cgr_slam::MotionModel::drawFromMotionEKFLinearized(const Pose2D& p,
                                                    const Pose2D& pnew, const Pose2D& pold){
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



GMapping::OrientedPoint cgr_slam::MotionModel::drawFromMotionTrueModel(const Pose2D& p, const Pose2D& pnew, const Pose2D& pold) {
  // TODO -- Required Unit testing
  // NOTE: AMCLOdomData *ndata
  double delta_rot1, delta_trans, delta_rot2;
  double delta_rot1_hat, delta_trans_hat, delta_rot2_hat;
  double delta_rot1_noise, delta_rot2_noise;

  // Avoid computing a bearing from two poses that are extremely near each
  // other (happens on in-place rotation).
  Pose2D delta=absoluteDifference(pnew, pold);
  //if(sqrt(ndata->delta.v[1]*ndata->delta.v[1] +
  //    ndata->delta.v[0]*ndata->delta.v[0]) < 0.01)
  if(sqrt(delta.x*delta.x + delta.y*delta.y) < 0.01){
    delta_rot1 = 0.0;
  }
  else{
    //delta_rot1 = angle_diff(atan2(ndata->delta.v[1], ndata->delta.v[0]), old_pose.v[2]);
    delta_rot1 = angle_diff(atan2(delta.y, delta.x), pold.theta);
  }

  //delta_trans = sqrt(ndata->delta.v[0]*ndata->delta.v[0] + ndata->delta.v[1]*ndata->delta.v[1]);
  delta_trans = sqrt(delta.x*delta.x + delta.y*delta.y);
  delta_rot2 = angle_diff(delta.theta, delta_rot1);

  // We want to treat backward and forward motion symmetrically for the
  // noise model to be applied below.  The standard model seems to assume
  // forward motion.
  delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1,0.0)),
                              fabs(angle_diff(delta_rot1,M_PI)));
  delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2,0.0)),
                              fabs(angle_diff(delta_rot2,M_PI)));
/*
  for (int i = 0; i < set->sample_count; i++)
  {
    pf_sample_t* sample = set->samples + i;

    // Sample pose differences
    delta_rot1_hat = angle_diff(delta_rot1,
                                pf_ran_gaussian(this->alpha1*delta_rot1_noise*delta_rot1_noise +
                                    this->alpha2*delta_trans*delta_trans));
    delta_trans_hat = delta_trans -
        pf_ran_gaussian(this->alpha3*delta_trans*delta_trans +
            this->alpha4*delta_rot1_noise*delta_rot1_noise +
            this->alpha4*delta_rot2_noise*delta_rot2_noise);
    delta_rot2_hat = angle_diff(delta_rot2,
                                pf_ran_gaussian(this->alpha1*delta_rot2_noise*delta_rot2_noise +
                                    this->alpha2*delta_trans*delta_trans));

    // Apply sampled update to particle pose
    sample->pose.v[0] += delta_trans_hat *
        cos(sample->pose.v[2] + delta_rot1_hat);
    sample->pose.v[1] += delta_trans_hat *
        sin(sample->pose.v[2] + delta_rot1_hat);
    sample->pose.v[2] += delta_rot1_hat + delta_rot2_hat;
  }
*/

  // Sample pose differences
  delta_rot1_hat = angle_diff(delta_rot1, GMapping::sampleGaussian(stt_*delta_rot1_noise*delta_rot1_noise +
                                  srt_*delta_trans*delta_trans));
  delta_trans_hat = delta_trans -
      GMapping::sampleGaussian(srr_*delta_trans*delta_trans +
          str_*delta_rot1_noise*delta_rot1_noise +
          str_*delta_rot2_noise*delta_rot2_noise);

  delta_rot2_hat = angle_diff(delta_rot2, GMapping::sampleGaussian(stt_*delta_rot2_noise*delta_rot2_noise +
                                  srt_*delta_trans*delta_trans));
  GMapping::OrientedPoint sampled_pose(0,0,0);
  sampled_pose.x = p.x + delta_trans_hat * cos(p.theta + delta_rot1_hat);
  sampled_pose.y = p.y + delta_trans_hat * sin(p.theta + delta_rot1_hat);
  sampled_pose.theta = delta_rot1_hat + delta_rot2_hat;
  return sampled_pose;
}

