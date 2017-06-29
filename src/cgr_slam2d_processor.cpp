#include "cgr_slam2d_processor.h"

using namespace cgr_slam;


CgrSlam2DProcessor::Particle::Particle(const cgr_slam::HMap2D &map) :
    map_(map), pose_(0,0,0), weight_(0), weight_sum_(0), gweight_(0), previous_index_(0){
  node_=0;
}

CgrSlam2DProcessor::TrajectoryNode::TrajectoryNode(const Pose2D& p, double w, TrajectoryNode* n, unsigned int c){
  pose_=p;
  weight_=w;
  childs_=c;
  parent_=n;
  reading_=0;
  gweight_=0;
  if (n){
    n->childs_++;
  }
  flag_=0;
  acc_weight_=0;
}


CgrSlam2DProcessor::TrajectoryNode::~TrajectoryNode(){
  if (parent_ && (--parent_->childs_)<=0)
    delete parent_;
  assert(!childs_);
}

CgrSlam2DProcessor::CgrSlam2DProcessor(int num_particles,
                                       double x_size_min,
                                       double x_size_max,
                                       double y_size_min,
                                       double y_size_max,
                                       double resol,
                                       Pose2D init_pose) {
  num_particles_ = num_particles;
  x_size_min_ = x_size_min;
  x_size_max_ = x_size_max;
  y_size_min_ = y_size_min;
  y_size_max_ = y_size_max;
  resol_ = resol;
  initial_pose_ = init_pose;
  //is_init_ = true;
  //motion_model_ = boost::shared_ptr<MotionModel>(new MotionModel());
  //Draw Random seed from sys time set seed once
  GMapping::sampleGaussian(1, time(NULL));

  resetAndBuildParticles();
}

void CgrSlam2DProcessor::resetAndBuildParticles() {
  particles_.clear();
  TrajectoryNode* node= new TrajectoryNode(initial_pose_, 0, 0, 0);
  HMap2D lmap(GMapping::Point(x_size_min_+x_size_max_, y_size_min_+y_size_max_)*.5, x_size_max_-x_size_min_,
              y_size_max_-y_size_min_, resol_);
  for (unsigned int i=0; i<num_particles_ ; i++){
    particles_.push_back(Particle(lmap));
    particles_.back().pose_=initial_pose_;
    particles_.back().previous_pose_=initial_pose_;
    particles_.back().setWeight(0);
    particles_.back().previous_index_=0;

    // we use the root directly
    particles_.back().node_= node;
  }

  // Create Cache for CGR SLAM
  q0.clear();
  q0_lik.clear();
  qr.clear();
  qr_lik.clear();
  q0.resize(num_particles_, Pose2D(0,0,0));
  qr.resize(num_particles_, Pose2D(0,0,0));
  q0_lik.resize(num_particles_, 0.0);
  qr_lik.resize(num_particles_, 0.0);
}

void CgrSlam2DProcessor::setMatchingParameters (double urange, double range, double sigma, int kernsize, double lopt, double aopt,
                                               int iterations, double likelihoodSigma, unsigned int likelihoodSkip){
  //m_obsSigmaGain=likelihoodGain;
  matcher_.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
  max_icp_iter_ = iterations;
}

void CgrSlam2DProcessor::setLaserConfig(LaserScanSensor *laser) {

  GMapping::SensorMap smap;
  smap.insert(std::make_pair(laser->getName(), laser));
  /*
  Construct the angle table for the sensor
  FIXME For now detect the readings of only the front laser, and assume its pose is in the center of the robot
  */
  GMapping::SensorMap::const_iterator laser_it = smap.find(std::string("FLASER"));
  if (laser_it == smap.end()) {
    std::cerr << "Attempting to load the new carmen log format" << std::endl;
    laser_it = smap.find(std::string("ROBOTLASER1"));
    assert(laser_it != smap.end());
  }
  const GMapping::RangeSensor *rangeSensor = dynamic_cast<const GMapping::RangeSensor *>((laser_it->second));
  assert(rangeSensor && rangeSensor->beams().size());

  num_laser_beams_ = static_cast<unsigned int>(rangeSensor->beams().size());
  double *angles = new double[rangeSensor->beams().size()];
  for (unsigned int i = 0; i < num_laser_beams_; i++) {
    angles[i] = rangeSensor->beams()[i].pose.theta;
  }
  // TODO -- Set Laser Params
  matcher_.setLaserParameters(num_laser_beams_, angles, rangeSensor->getPose());
  delete[] angles;
}

bool CgrSlam2DProcessor::setDiffDriveParams(double srr, double srt, double str, double stt) {
  motion_model_.setParams(srr,srt,str,stt);
  return true;
}

// ---------- Main ------
/* Credit OpenSlam GMapping gsp_->processScan()*/
bool CgrSlam2DProcessor::processReading(Scan &reading) {
  /**retireve the position from the reading, and compute the odometry*/
  Pose2D relPose = reading.getPose();
  if (!is_first_scan_received_) {
    last_part_pose_ = odom_pose_ = relPose;
  }

  // Draw sample for every incoming scan
  //write the state of the reading and update all the particles using the motion model
  // CGR/GMapping: Particle Filter Predict Step
  if(update_motion_frequent_)
    performPredictStep(relPose);

  // accumulate the robot translation and rotation
  Pose2D move = relPose - odom_pose_;
  move.theta = atan2(sin(move.theta), cos(move.theta));
  acc_linear_distance_ += sqrt(move * move);
  acc_angular_distance_ += fabs(move.theta);

  // if the robot jumps throw a warning
  if (acc_linear_distance_ > dist_th_check_) {
    LOGPRINT_WARN("The Robot Jumps too far for this reading");
  }

  odom_pose_ = relPose;

  bool processed = false;

  // process a scan only if the robot has traveled a given distance or a certain amount of time has elapsed
  if (!is_first_scan_received_
      || acc_linear_distance_ >= linear_update_dist_th_
      || acc_angular_distance_ >= angular_update_dist_th_
      || (temporal_update_time_th_ >= 0.0 && (reading.getTime() - last_update_time_) > temporal_update_time_th_)) {
    last_update_time_ = reading.getTime();

    //this is for converting the reading in a scan-matcher feedable form
    assert(reading.size() == num_laser_beams_);
    double *plainReading = new double[num_laser_beams_];
    for (unsigned int i = 0; i < num_laser_beams_; i++) {
      plainReading[i] = reading[i];
    }

    Scan *reading_copy =
        new Scan(reading.size(),
                 &(reading[0]),
                 static_cast<const LaserScanSensor*>(reading.getSensor()),
                 reading.getTime());


    if(!update_motion_frequent_)
      performPredictStep(relPose);

    if (is_first_scan_received_) {
      if(use_gmapping_){
        // Gmapping Algorithm
        scanMatch(plainReading);
      }
      else{
        // Corrective Gradient Refinement Algorithm
        // CGR: Refine & Acceptance Test Step
        performRefineAndAccept(plainReading);
      }
      // Will use Selective Resampling from Gmapping
      updateTreeWeights(false);
      resample(plainReading, false, reading_copy);

    } else {

      LOGPRINT_WARN("Registering First Scan");
      for (ParticleVector::iterator it = particles_.begin(); it != particles_.end(); it++) {
        matcher_.invalidateActiveArea();
        matcher_.computeActiveArea(it->map_, it->pose_, plainReading);
        matcher_.registerScan(it->map_, it->pose_, plainReading);

        // cyr: not needed anymore, particles refer to the root in the beginning!
        TrajectoryNode *node = new TrajectoryNode(it->pose_, 0., it->node_, 0);
        //node->reading=0;
        node->reading_ = reading_copy;
        it->node_ = node;

      }
      is_first_scan_received_ = true;
    }
    //		cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
    updateTreeWeights(false);
    //		cerr  << ".done!" <<endl;

    delete[] plainReading;
    last_part_pose_ = odom_pose_; //update the past pose for the next iteration
    acc_linear_distance_ = 0;
    acc_angular_distance_ = 0;

    processed = true;

    //keep ready for the next step
    for (ParticleVector::iterator it = particles_.begin(); it != particles_.end(); it++) {
      it->previous_pose_ = it->pose_;
    }

  }
  //m_readingCount++;
  return processed;
}

void CgrSlam2DProcessor::performPredictStep(Pose2D& relPose) {
  if(true_diff_drive_motion_model_) {
    //LOGPRINT_DEBUG("TRUE DIFF DRIVE*******");
    for (ParticleVector::iterator it = particles_.begin(); it != particles_.end(); it++) {
      Pose2D &pose(it->pose_);
      pose = motion_model_.drawFromMotionTrueModel(it->pose_, relPose, odom_pose_);
    }
  }
  else{
    for (ParticleVector::iterator it = particles_.begin(); it != particles_.end(); it++) {
      Pose2D &pose(it->pose_);
      pose = motion_model_.drawFromMotionEKFLinearized(it->pose_, relPose, odom_pose_);
    }
  }
}



/// ALGORITHMS


/**Just scan match every single particle.
If the scan matching fails, the particle gets a default likelihood.*/
void CgrSlam2DProcessor::scanMatch(const double* plainReading){
  // sample a new pose from each scan in the reference

  double sumScore=0;

  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    Pose2D corrected;
    double score, l, s;
    score=matcher_.optimize(corrected, it->map_, it->pose_, plainReading);
    //    it->pose=corrected;
    //double minimum_score = 50.0;
    if (score > minimum_score_){
      it->pose_=corrected;
    } else {
      //LOGPRINT_DEBUG("ParticleID[%d]: (ScanMatch < minimum_score)= %lf, %lf, Use odom pose instead",
      //               (int)(it - particles_.begin()), score, minimum_score_);
    }

    matcher_.likelihoodAndScore(s, l, it->map_, it->pose_, plainReading);
    sumScore+=score;
    it->weight_+=l;
    it->weight_sum_+=l;

    //set up the selective copy of the active area
    //by detaching the areas that will be updated
    matcher_.invalidateActiveArea();
    matcher_.computeActiveArea(it->map_, it->pose_, plainReading);
  }
  //if (m_infoStream)
  //  m_infoStream << "Average Scan Matching Score=" << sumScore/particles_.size() << std::endl;
}

void CgrSlam2DProcessor::normalize(){
  //normalize the log m_weights
  double gain=1./(o_sigma_gain_*particles_.size());
  double lmax= -std::numeric_limits<double>::max();
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    lmax=it->weight_>lmax?it->weight_:lmax;
  }
  //cout << "!!!!!!!!!!! maxwaight= "<< lmax << endl;

  weights_.clear();
  double wcum=0;
  m_neff=0;
  for (std::vector<Particle>::iterator it=particles_.begin(); it!=particles_.end(); it++){
    weights_.push_back(exp(gain*(it->weight_-lmax)));
    wcum+=weights_.back();
    //cout << "l=" << it->weight<< endl;
  }

  m_neff=0;
  for (std::vector<double>::iterator it=weights_.begin(); it!=weights_.end(); it++){
    *it=*it/wcum;
    double w=*it;
    m_neff+=w*w;
  }
  m_neff=1./m_neff;

}

bool CgrSlam2DProcessor::resample(const double* plainReading, int adaptSize, const Scan* reading){

  bool hasResampled = false;

  TrajectoryNodeVector oldGeneration;
  for (unsigned int i=0; i<particles_.size(); i++){
    oldGeneration.push_back(particles_[i].node_);
  }

  if (m_neff<resample_th_*particles_.size()){
    LOGPRINT_INFO("Resampling, current neff=%lf", m_neff);

    uniform_resampler<double, double> resampler;
    indexes_=resampler.resampleIndexes(weights_, adaptSize);


    //BEGIN: BUILDING TREE
    ParticleVector temp;
    unsigned int j=0;
    std::vector<unsigned int> deletedParticles;  		//this is for deleteing the particles which have been resampled away.

    //		cerr << "Existing Nodes:" ;
    for (unsigned int i=0; i<indexes_.size(); i++){
      //			cerr << " " << m_indexes[i];
      while(j<indexes_[i]){
        deletedParticles.push_back(j);
        j++;
      }
      if (j==indexes_[i])
        j++;
      Particle & p=particles_[indexes_[i]];
      TrajectoryNode* node=0;
      TrajectoryNode* oldNode=oldGeneration[indexes_[i]];
      //			cerr << i << "->" << m_indexes[i] << "B("<<oldNode->childs <<") ";
      node=new	TrajectoryNode(p.pose_, 0, oldNode, 0);
      //node->reading=0;
      node->reading_=reading;
      //			cerr << "A("<<node->parent->childs <<") " <<endl;

      temp.push_back(p);
      temp.back().node_=node;
      temp.back().previous_index_=indexes_[i];
    }
    while(j<indexes_.size()){
      deletedParticles.push_back(j);
      j++;
    }
    //		cerr << endl;
    // std::cerr <<  "Deleting Nodes:";
    for (unsigned int i=0; i<deletedParticles.size(); i++){
      // std::cerr <<" " << deletedParticles[i];
      delete particles_[deletedParticles[i]].node_;
      particles_[deletedParticles[i]].node_=0;
    }
    // std::cerr  << " Done" <<std::endl;

    //END: BUILDING TREE
    // std::cerr << "Deleting old particles..." ;
    particles_.clear();
    // std::cerr << "Done" << std::endl;
    // std::cerr << "Copying Particles and  Registering  scans...";
    for (ParticleVector::iterator it=temp.begin(); it!=temp.end(); it++){
      it->setWeight(0);
      matcher_.invalidateActiveArea();
      matcher_.registerScan(it->map_, it->pose_, plainReading);
      particles_.push_back(*it);
    }
    // std::cerr  << " Done" <<std::endl;
    hasResampled = true;
  } else {
    int index=0;
    // std::cerr << "Registering Scans:";
    TrajectoryNodeVector::iterator node_it=oldGeneration.begin();
    for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
      //create a new node in the particle tree and add it to the old tree
      //BEGIN: BUILDING TREE
      TrajectoryNode* node=0;
      node=new TrajectoryNode(it->pose_, 0.0, *node_it, 0);

      //node->reading=0;
      node->reading_=reading;
      it->node_=node;

      //END: BUILDING TREE
      matcher_.invalidateActiveArea();
      matcher_.registerScan(it->map_, it->pose_, plainReading);
      it->previous_index_=index;
      index++;
      node_it++;

    }

  }
  //END: BUILDING TREE

  return hasResampled;
}





/// TREE IMPLEMENTATION & PROCESSING
//BEGIN State Save/Restore

CgrSlam2DProcessor::TrajectoryNodeVector CgrSlam2DProcessor::getTrajectories() const {
  TrajectoryNodeVector v;
  TrajectoryNodeMultimap parentCache;
  TrajectoryNodeDeque border;

  for (ParticleVector::const_iterator it=particles_.begin(); it!=particles_.end(); it++){
    TrajectoryNode* node=it->node_;
    while(node){
      node->flag_=false;
      node=node->parent_;
    }
  }

  for (ParticleVector::const_iterator it=particles_.begin(); it!=particles_.end(); it++){
    TrajectoryNode* newnode=new TrajectoryNode(* (it->node_) );

    v.push_back(newnode);
    assert(newnode->childs_==0);
    if (newnode->parent_){
      parentCache.insert(std::make_pair(newnode->parent_, newnode));
      //cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
      if (! newnode->parent_->flag_){
        //cerr << __PRETTY_FUNCTION__ << ": node " << newnode->parent << " flag=" << newnode->parent->flag<< endl;
        newnode->parent_->flag_=true;
        border.push_back(newnode->parent_);
      }
    }
  }

  //cerr << __PRETTY_FUNCTION__ << ": border.size(INITIAL)=" << border.size() << endl;
  //cerr << __PRETTY_FUNCTION__ << ": parentCache.size()=" << parentCache.size() << endl;
  while (! border.empty()){
    //cerr << __PRETTY_FUNCTION__ << ": border.size(PREPROCESS)=" << border.size() << endl;
    //cerr << __PRETTY_FUNCTION__ << ": parentCache.size(PREPROCESS)=" << parentCache.size() << endl;
    const TrajectoryNode* node=border.front();
    //cerr << __PRETTY_FUNCTION__ << ": node " << node << endl;
    border.pop_front();
    if (! node)
      continue;

    TrajectoryNode* newnode=new TrajectoryNode(*node);
    node->flag_=false;

    //update the parent of all of the referring childs
    std::pair<TrajectoryNodeMultimap::iterator, TrajectoryNodeMultimap::iterator> p=parentCache.equal_range(node);
    double childs=0;
    for (TrajectoryNodeMultimap::iterator it=p.first; it!=p.second; it++){
      assert(it->second->parent_==it->first);
      (it->second)->parent_=newnode;
      //cerr << "PS(" << it->first << ", "<< it->second << ")";
      childs++;
    }
    ////cerr << endl;
    parentCache.erase(p.first, p.second);
    //cerr << __PRETTY_FUNCTION__ << ": parentCache.size(POSTERASE)=" << parentCache.size() << endl;
    assert(childs==newnode->childs_);

    //unmark the node
    if ( node->parent_ ){
      parentCache.insert(std::make_pair(node->parent_, newnode));
      if(! node->parent_->flag_){
        border.push_back(node->parent_);
        node->parent_->flag_=true;
      }
    }
    //insert the parent in the cache
  }
  //cerr << __PRETTY_FUNCTION__ << " : checking cloned trajectories" << endl;
  for (unsigned int i=0; i<v.size(); i++){
    TrajectoryNode* node= v[i];
    while (node){
      //cerr <<".";
      node=node->parent_;
    }
    //cerr << endl;
  }

  return v;

}

void CgrSlam2DProcessor::integrateScanSequence(TrajectoryNode* node){
  //reverse the list
  TrajectoryNode* aux=node;
  TrajectoryNode* reversed=0;
  double count=0;
  while(aux!=0){
    TrajectoryNode * newnode=new TrajectoryNode(*aux);
    newnode->parent_=reversed;
    reversed=newnode;
    aux=aux->parent_;
    count++;
  }



  aux=reversed;
  bool first=true;
  double oldWeight=0;
  Pose2D oldPose;
  while (aux!=0){
    if (first){
      oldPose=aux->pose_;
      first=false;
      oldWeight=aux->weight_;
    }

    Pose2D dp=aux->pose_-oldPose;
    double dw=aux->weight_-oldWeight;
    oldPose=aux->pose_;


    double * plainReading = new double[num_laser_beams_];
    for(unsigned int i=0; i<num_laser_beams_; i++)
      plainReading[i]=(*(aux->reading_))[i];

    for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
      //compute the position relative to the path;
      double s=sin(oldPose.theta-it->pose_.theta),
          c=cos(oldPose.theta-it->pose_.theta);

      it->pose_.x+=c*dp.x-s*dp.y;
      it->pose_.y+=s*dp.x+c*dp.y;
      it->pose_.theta+=dp.theta;
      it->pose_.theta=atan2(sin(it->pose_.theta), cos(it->pose_.theta));

      //register the scan
      matcher_.invalidateActiveArea();
      matcher_.computeActiveArea(it->map_, it->pose_, plainReading);
      it->weight_+=dw;
      it->weight_sum_+=dw;

      // this should not work, since it->weight is not the correct weight!
      //			it->node=new TrajectoryNode(it->pose, it->weight, it->node);
      it->node_=new TrajectoryNode(it->pose_, 0.0, it->node_);
      //update the weight
    }

    delete [] plainReading;
    aux=aux->parent_;
  }

  //destroy the path
  aux=reversed;
  while (reversed){
    aux=reversed;
    reversed=reversed->parent_;
    delete aux;
  }
}

//END State Save/Restore

//BEGIN

void  CgrSlam2DProcessor::updateTreeWeights(bool weightsAlreadyNormalized){

  if (!weightsAlreadyNormalized) {
    normalize();
  }
  resetTree();
  propagateWeights();
}

void CgrSlam2DProcessor::resetTree(){
  // don't calls this function directly, use updateTreeWeights(..) !

  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    TrajectoryNode* n=it->node_;
    while (n){
      n->acc_weight_=0;
      n->visit_counter_=0;
      n= n->parent_;
    }
  }
}

double CgrSlam2DProcessor::propagateWeight(TrajectoryNode* n, double weight){
  if (!n)
    return weight;
  double w=0;
  n->visit_counter_++;
  n->acc_weight_+=weight;
  if (n->visit_counter_==n->childs_){
    w=propagateWeight(n->parent_,n->acc_weight_);
  }
  assert(n->visit_counter_<= n->childs_);
  return w;
}

double CgrSlam2DProcessor::propagateWeights(){
  // don't calls this function directly, use updateTreeWeights(..) !

  // all nodes must be resetted to zero and weights normalized

  // the accumulated weight of the root
  double lastNodeWeight=0;
  // sum of the weights in the leafs
  double aw=0;

  std::vector<double>::iterator w=weights_.begin();
  for (ParticleVector::iterator it=particles_.begin(); it!=particles_.end(); it++){
    double weight=*w;
    aw+=weight;
    TrajectoryNode* n = it->node_;
    n->acc_weight_=weight;
    lastNodeWeight+=propagateWeight(n->parent_,n->acc_weight_);
    w++;
  }

   /*
  if (fabs(aw-1.0) > 0.0001 || fabs(lastNodeWeight-1.0) > 0.0001) {
    cerr << "ERROR: ";
    cerr << "root->accWeight=" << lastNodeWeight << "    sum_leaf_weights=" << aw << endl;
    assert(0);
  }
    */
  return lastNodeWeight;
}

int CgrSlam2DProcessor::getBestParticleIndex() const{
  unsigned int bi=0;
  double bw=-std::numeric_limits<double>::max();
  for (unsigned int i=0; i<particles_.size(); i++)
    if (bw<particles_[i].weight_sum_){
      bw=particles_[i].weight_sum_;
      bi=i;
    }
  return (int) bi;
}

// Corrective Gradient Refinement Related

void CgrSlam2DProcessor::performRefineAndAccept(const double *plainReading) {

  // CGR REFINE STEP
  for(int i=0; i < particles_.size(); i++){ // For each particle
    Pose2D last_pose = particles_[i].pose_;
    Pose2D out_pose(0,0,0);
    double icp_err = 0.;
    //q0_lik[i] = matcher_.score(particles_[i].map_, last_pose, plainReading);
    double score=0.0, lik=0.0;
    matcher_.likelihoodAndScore(score, lik, particles_[i].map_, last_pose, plainReading);
    q0_lik[i] = lik;
    q0[i] = particles_[i].pose_;

    for(int j=0; j < max_icp_iter_; j++){

      if(non_linear_icp_)
        icp_err = matcher_.computeIcpNonLinearStep(out_pose, particles_[i].map_, last_pose, plainReading);
      else
        icp_err = matcher_.computeIcpLinearStep(out_pose, particles_[i].map_, last_pose, plainReading);
      // TODO -- Collect Cache to check convergence
      if (icp_err == std::numeric_limits<double>::infinity()){
        LOGPRINT_WARN("Particle %d will not converge -- Break & use latest pose", i);
        out_pose = last_pose;
        break;
      }
      else{
        last_pose = out_pose;
      }

    }
    //LOGPRINT_DEBUG("Particle %d: Final ICP Error=%lf", i, icp_err);
    qr[i] = out_pose;
    score = 0.0; lik=0.0;
    // TODO -- GMapping Uses log likelihood -> we might have to change to normalized real likelihood for algorithm eval
    matcher_.likelihoodAndScore(score, lik, particles_[i].map_, out_pose, plainReading);
    qr_lik[i] = lik;

    // CGR Acceptance Test and Compute Active Map Area
    if (qr_lik[i] > q0_lik[i]){
      particles_[i].pose_ = out_pose;
    }
    else{
      LOGPRINT_WARN("Particle %d: Acceptance Test fail, trust Odom", i);
    }
    //LOGPRINT_DEBUG("Particle %d: Acceptance Ratio r_%d=%lf, qr_likelihood=%lf, q0_likelihood=%lf", i,
    //               i, std::min(1.0,qr_lik[i]/q0_lik[i]), qr_lik[i], q0_lik[i]);


    matcher_.invalidateActiveArea();
    matcher_.computeActiveArea(particles_[i].map_, particles_[i].pose_, plainReading);
  }

}

