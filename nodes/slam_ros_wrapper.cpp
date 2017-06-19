#include "slam_ros_wrapper.h"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

SlamRosWrapper::SlamRosWrapper():
  pnh_("~"),
  spinner_(1)
{
  // --- ROS Node params --
  pnh_.param("tf_publish_period", tf_publish_period_, 0.05);
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
  pnh_.param<std::string>("base_frame", robot_frame_, "base_link");
  pnh_.param("map_render_interval", map_render_interval_, 5.0);
  // force_render: Render every map_render_interval even robot move below linear/ang dist update
  pnh_.param("force_render", force_render_, false);
  entropy_publisher_ = pnh_.advertise<std_msgs::Float64>("entropy", 1, true);
  sst_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = nh_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

  // Init ROS Publisher/Subscriber
  scan_filter_sub_ = boost::shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan> >
      (new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, "scan", 5));
  scan_filter_ = boost::shared_ptr<tf::MessageFilter<sensor_msgs::LaserScan> >
      (new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5));
  scan_filter_->registerCallback(boost::bind(&SlamRosWrapper::laserCallback, this, _1));

  initProcessorInstance();
}

SlamRosWrapper::~SlamRosWrapper(){
  delete(laser_sensor_);
  //TODO -- Destroy thread properly if first Scan is not received
  if(!is_first_scan_received_)
    wait_first_scan_signal_.notify_all();


  spinner_.stop();
  LOGPRINT_DEBUG("-- DONE SHUTDOWN CGR --");

}

void SlamRosWrapper::executeNode() {
  ros::Rate rate(1.0/tf_publish_period_);

  spinner_.start(); // Start Laser Subscriber
  {
    boost::mutex::scoped_lock lock(wait_first_scan_mutex_);
    // TODO -- ADD Condition for ros::ok()!!
    wait_first_scan_signal_.wait(lock);
  }

#ifdef ENABLE_MAP_RENDERER_THREAD
  if (ros::ok() && is_first_scan_received_){
    // Register Map renderer if first scan received
    map_renderer_thread_ = boost::shared_ptr<boost::thread>
        (new boost::thread(boost::bind(&SlamRosWrapper::mapRendererThread, this)));
  }
#endif

  ROS_WARN("---------------- Start TF Publisher Thread -----------------");
  while (ros::ok()){
    broadcastMapTransform();
    //ROS_WARN("PUB MAP");
    rate.sleep();
  }
  spinner_.stop();
#ifdef ENABLE_MAP_RENDERER_THREAD
  LOGPRINT_DEBUG("WAIT MAP RENDER THREAD TO SHUTDOWN");
  map_renderer_thread_->join();
#endif
}


bool SlamRosWrapper::getOdomToRobotLaser(cgr_slam::Pose2D &odom_laser_pose, ros::Time t) {
  // Get the pose of the centered laser at the right time
  centered_laser_pose_.stamp_ = t;
  // Get the laser's pose that is centered
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, centered_laser_pose_, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  odom_laser_pose = cgr_slam::Pose2D(odom_pose.getOrigin().x(),
                                      odom_pose.getOrigin().y(),
                                      yaw);
  return true;
}

bool SlamRosWrapper::processCgrSlam(const sensor_msgs::LaserScan& scan, cgr_slam::Pose2D& odom_pose){
  if(!getOdomToRobotLaser(odom_pose, scan.header.stamp))
    return false;

  if(scan.ranges.size() != laser_total_beam_count_)
    return false;

  // GMapping wants an array of doubles...
  double* ranges_double = new double[scan.ranges.size()];
  // If the angle increment is negative, we have to invert the order of the readings.
  if (do_reverse_range_)
  {
    ROS_DEBUG("Inverting scan");
    int num_ranges = scan.ranges.size();
    for(int i=0; i < num_ranges; i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[num_ranges - i - 1] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[num_ranges - i - 1];
    }
  } else
  {
    for(unsigned int i=0; i < scan.ranges.size(); i++)
    {
      // Must filter out short readings, because the mapper won't
      if(scan.ranges[i] < scan.range_min)
        ranges_double[i] = (double)scan.range_max;
      else
        ranges_double[i] = (double)scan.ranges[i];
    }
  }

  GMapping::RangeReading reading(scan.ranges.size(),
                                 ranges_double,
                                 laser_sensor_,
                                 scan.header.stamp.toSec());

  // ...but it deep copies them in RangeReading constructor, so we don't
  // need to keep our array around.
  delete[] ranges_double;

  reading.setPose(odom_pose);

  /*
  ROS_DEBUG("scanpose (%.3f): %.3f %.3f %.3f\n",
            scan.header.stamp.toSec(),
            gmap_pose.x,
            gmap_pose.y,
            gmap_pose.theta);
            */
  ROS_DEBUG("processing scan");

  return slam_proc_->processReading(reading);
}

void SlamRosWrapper::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  // TODO -- Throttle Scans
  // static ros::Time last_map_update(0,0);

  if (!is_first_scan_received_){
    if(initNodeFromFirstScan(*scan)){
      ROS_INFO("RECEIVE FIRST SCAN, ... Start CGR SLAM !!!!!");
      wait_first_scan_signal_.notify_all();
      is_first_scan_received_ = true;
      last_map_update_ = ros::Time(0,0);
    }
  }

  cgr_slam::Pose2D odom_pose;

  // For safety ...
  slam_mutex_.lock();
  bool process_success = processCgrSlam(*scan, odom_pose);
  slam_mutex_.unlock();

  if(process_success){
    ROS_DEBUG("Scan Processed");
    slam_mutex_.lock();
    cgr_slam::Pose2D mpose = slam_proc_->getParticles()[slam_proc_->getBestParticleIndex()].pose_;
    slam_mutex_.unlock();
    //cgr_slam::Pose2D mpose(0.0, 0.0, 0.0);

    ROS_DEBUG("new best pose: %.3f %.3f %.3f", mpose.x, mpose.y, mpose.theta);
    ROS_DEBUG("odom pose: %.3f %.3f %.3f", odom_pose.x, odom_pose.y, odom_pose.theta);
    ROS_DEBUG("correction: %.3f %.3f %.3f", mpose.x - odom_pose.x, mpose.y - odom_pose.y, mpose.theta - odom_pose.theta);

    tf::Transform laser_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta), tf::Vector3(mpose.x, mpose.y, 0.0)).inverse();
    tf::Transform odom_to_laser = tf::Transform(tf::createQuaternionFromRPY(0, 0, odom_pose.theta), tf::Vector3(odom_pose.x, odom_pose.y, 0.0));

    {
      boost::mutex::scoped_lock lock(tf_map_to_odom_mutex_);
      tf_map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
    }


    if((scan->header.stamp - last_map_update_).toSec() > map_render_interval_)
    {
      //boost::mutex::scoped_lock lock_render_signal(render_map_mutex_);
      // Separate Thread to speed up map creation process
#ifndef ENABLE_MAP_RENDERER_THREAD
      updateMap();
#endif
#ifdef ENABLE_MAP_RENDERER_THREAD
      wait_render_map_signal_.notify_one();
#endif
      last_map_update_ = scan->header.stamp;
    }

  }
  else{
    ROS_DEBUG("--SKIP This Process Scan--");
  }
}

void SlamRosWrapper::mapRendererThread() {
  /*{
    //boost::mutex::scoped_lock lock(wait_first_scan_mutex_);
    //wait_first_scan_signal_.wait(lock, boost::function<bool (void)>(boost::bind(&SlamRosWrapper::isRosShutdown, this)));
    //ROS_WARN("Invoke Map Renderer, Period: %lf s", map_render_interval_);
  }*/
  ROS_WARN("Invoke Map Renderer, Period: %lf s", map_render_interval_);
  ros::Rate render_rate(5.0);
  while(ros::ok()){
    // WAIT SIGNAL
    boost::system_time const timeout=boost::get_system_time()+ boost::posix_time::seconds(map_render_interval_);
    //boost::cv_status wait_status = boost::cv_status::timeout;
    bool wait_status;
    {
      boost::mutex::scoped_lock lock(render_map_mutex_);
      // wait_status = wait_render_map_signal_.wait(lock);
      wait_status = wait_render_map_signal_.timed_wait(lock, timeout);
    }

    if (wait_status || force_render_){
      ROS_DEBUG("Rendering Map....., in THREAD");
      updateMap();

    }
    else{
      // Waiting longer than map_render_interval_, should never happened ...
      ROS_DEBUG("Rendering Map signal timeout ...");
    }

    render_rate.sleep();
  }
}

void SlamRosWrapper::updateMap() {
  ROS_DEBUG("Update map");
  boost::mutex::scoped_lock map_lock (map_mutex_);
  GMapping::ScanMatcher matcher;

  matcher.setLaserParameters(laser_total_beam_count_, &(laser_angles_[0]),
                             laser_sensor_->getPose());

  matcher.setlaserMaxRange(laser_max_range_);
  matcher.setusableRange(laser_max_usable_range_);
  matcher.setgenerateMap(true);


  if(!got_map_) {
    map_.map.info.resolution = map_resol_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  GMapping::Point center;
  center.x=(x_min_size_ + x_max_size_) / 2.0;
  center.y=(y_min_size_ + y_max_size_) / 2.0;

  GMapping::ScanMatcherMap smap(center, x_min_size_, x_max_size_, y_min_size_, y_max_size_,
                                map_resol_);


  slam_mutex_.lock();
  //TODO -- If possible, fast deep copy first, register later
  // Lock Slam Trajectory Data
  cgr_slam::CgrSlam2DProcessor::Particle best = slam_proc_->getParticles()[slam_proc_->getBestParticleIndex()];

  std_msgs::Float64 entropy;
  entropy.data = computePoseEntropy();
  if(entropy.data > 0.0)
    entropy_publisher_.publish(entropy);

  ROS_DEBUG("Trajectory tree:");
  for(cgr_slam::CgrSlam2DProcessor::TrajectoryNode* n = best.node_;
      n;
      n = n->parent_)
  {
    ROS_DEBUG("  %.3f %.3f %.3f",
              n->pose_.x,
              n->pose_.y,
              n->pose_.theta);
    if(!n->reading_)
    {
      ROS_DEBUG("Reading is NULL");
      continue;
    }
    matcher.invalidateActiveArea();
    matcher.computeActiveArea(smap, n->pose_, &((*n->reading_)[0]));
    matcher.registerScan(smap, n->pose_, &((*n->reading_)[0]));
  }
  // UnLock Slam Trajectory Data
  slam_mutex_.unlock();


  // the map may have expanded, so resize ros message as well
  if(map_.map.info.width != (unsigned int) smap.getMapSizeX() || map_.map.info.height != (unsigned int) smap.getMapSizeY()) {

    // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
    //       so we must obtain the bounding box in a different way
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
    GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
    x_min_size_ = wmin.x; y_min_size_ = wmin.y;
    x_max_size_ = wmax.x; y_max_size_ = wmax.y;

    ROS_DEBUG("map size is now %dx%d pixels (%f,%f)-(%f, %f)", smap.getMapSizeX(), smap.getMapSizeY(),
              x_min_size_, y_min_size_, x_max_size_, y_max_size_);

    map_.map.info.width = smap.getMapSizeX();
    map_.map.info.height = smap.getMapSizeY();
    map_.map.info.origin.position.x = x_min_size_;
    map_.map.info.origin.position.y = y_min_size_;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);

    ROS_DEBUG("map origin: (%f, %f)", map_.map.info.origin.position.x, map_.map.info.origin.position.y);
  }

  for(int x=0; x < smap.getMapSizeX(); x++)
  {
    for(int y=0; y < smap.getMapSizeY(); y++)
    {
      /// @todo Sort out the unknown vs. free vs. obstacle thresholding
      GMapping::IntPoint p(x, y);
      double occ=smap.cell(p);
      assert(occ <= 1.0);
      if(occ < 0)
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
      else if(occ > occ_thresh_)
      {
        //map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = (int)round(occ*100.0);
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
      }
      else
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
    }
  }
  got_map_ = true;

  //make sure to set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = tf_.resolve( map_frame_ );

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

}

void SlamRosWrapper::broadcastMapTransform() {
  boost::mutex::scoped_lock lock(tf_map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + ros::Duration(tf_publish_period_);
  tf_bc_.sendTransform( tf::StampedTransform (tf_map_to_odom_, tf_expiration, map_frame_, odom_frame_));
}

bool SlamRosWrapper::initNodeFromFirstScan(const sensor_msgs::LaserScan &scan) {
  // Exactly copy from Gmapping ROS wrapper
  laser_frame_ = scan.header.frame_id;
  // Get the laser's pose, relative to base.
  tf::Stamped<tf::Pose> ident;
  tf::Stamped<tf::Transform> laser_pose;
  ident.setIdentity();
  ident.frame_id_ = laser_frame_;
  ident.stamp_ = scan.header.stamp;
  try
  {
    tf_.transformPose(robot_frame_, ident, laser_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
             e.what());
    return false;
  }

  // create a point 1m above the laser position and transform it into the laser-frame
  tf::Vector3 v;
  v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
  tf::Stamped<tf::Vector3> up(v, scan.header.stamp,
                              robot_frame_);
  try
  {
    tf_.transformPoint(laser_frame_, up, up);
    ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
  }
  catch(tf::TransformException& e)
  {
    ROS_WARN("Unable to determine orientation of laser: %s",
             e.what());
    return false;
  }

  // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
  if (fabs(fabs(up.z()) - 1) > 0.001)
  {
    ROS_WARN("Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f",
             up.z());
    return false;
  }

  laser_total_beam_count_ = scan.ranges.size();

  double angle_center = (scan.angle_min + scan.angle_max)/2;

  if (up.z() > 0)
  {
    do_reverse_range_ = scan.angle_min > scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(0,0,angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upwards.");
  }
  else
  {
    do_reverse_range_ = scan.angle_min < scan.angle_max;
    centered_laser_pose_ = tf::Stamped<tf::Pose>(tf::Transform(tf::createQuaternionFromRPY(M_PI,0,-angle_center),
                                                               tf::Vector3(0,0,0)), ros::Time::now(), laser_frame_);
    ROS_INFO("Laser is mounted upside down.");
  }

  // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
  laser_angles_.resize(scan.ranges.size());
  // Make sure angles are started so that they are centered
  double theta = - std::fabs(scan.angle_min - scan.angle_max)/2;
  for(unsigned int i=0; i<scan.ranges.size(); ++i)
  {
    laser_angles_[i]=theta;
    theta += std::fabs(scan.angle_increment);
  }

  ROS_INFO("Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f", scan.angle_min, scan.angle_max,
            scan.angle_increment);
  ROS_INFO("Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f", laser_angles_.front(),
            laser_angles_.back(), std::fabs(scan.angle_increment));

  GMapping::OrientedPoint gmap_pose(0, 0, 0);

  // setting maxRange and maxUrange here so we can set a reasonable default
  ros::NodeHandle private_nh_("~/laser");
  if(!private_nh_.getParam("max_range", laser_max_range_)){
    ROS_WARN("Use max range from first laser reading");
    laser_max_range_ = scan.range_max - 0.01;
  }

  if(!private_nh_.getParam("max_usable_range", laser_max_usable_range_)){
    ROS_WARN("Max Usable Range is not set");
    laser_max_usable_range_ = laser_max_range_;
  }


  laser_sensor_ = new cgr_slam::LaserScanSensor(
      "FLASER",
      laser_total_beam_count_,
      fabs(scan.angle_increment),
      gmap_pose,
      0.0,
      laser_max_range_
  );

  ROS_ASSERT(laser_sensor_);
  ROS_WARN("---------------DONE-----------------");
  // Init Laser Reading
  slam_proc_->setLaserConfig(laser_sensor_);
  // odom_sensor_ = boost::shared_ptr<cgr_slam::OdomSensor>(new cgr_slam::OdomSensor())

  /// @todo Expose setting an initial pose
  cgr_slam::Pose2D initialPose;
  if(!getOdomToRobotLaser(initialPose, scan.header.stamp))
  {
    ROS_WARN("Unable to determine inital pose of laser! Starting point will be set to zero.");
    initialPose = cgr_slam::Pose2D(0.0, 0.0, 0.0);
  }

  return initProcessorParams();
}

void SlamRosWrapper::initProcessorInstance() {

  // -- CGR SLAM params --
  ros::NodeHandle slam_nh(pnh_, "slam");
  slam_nh.param("linear_update", linear_u_dist_, 0.5);
  slam_nh.param("angular_update", angular_u_dist_, 0.5);
  slam_nh.param("particles", particles_, 30);
  // double x_min, x_max, y_min, y_max, map_resol;
  slam_nh.param("x_map_min", x_min_size_, -20.0);
  slam_nh.param("x_map_max", x_max_size_, 20.0);
  slam_nh.param("y_map_min", y_min_size_, -20.0);
  slam_nh.param("y_map_max", y_max_size_, 20.0);
  slam_nh.param("map_resol", map_resol_, 0.05);
  slam_nh.param("resample_threshold", resample_th_, 0.5);
  slam_nh.param("occ_threshold", occ_thresh_, 0.25);


  // TODO -- Make Motion model setup More Dynamic
  ros::NodeHandle mm_nh(pnh_, "motion_model");
  std::string mm_type;
  double srr, srt, str, stt;
  mm_nh.param<std::string>("type", mm_type, "diff_drive");
  mm_nh.param("srr", srr, 0.1);
  mm_nh.param("srt", srt, 0.2);
  mm_nh.param("str", str, 0.1);
  mm_nh.param("stt", stt, 0.2);


  slam_proc_ = boost::shared_ptr
      <cgr_slam::CgrSlam2DProcessor>(new cgr_slam::CgrSlam2DProcessor
                                         (particles_, x_min_size_, x_max_size_,
                                          y_min_size_, y_max_size_, map_resol_
                                          ));

  ROS_ASSERT_MSG(slam_proc_->setDiffDriveParams(srr, srt, str, stt),
                 "Fail to Set Motion Model, Something Wrong!");

  ROS_ASSERT_MSG(slam_proc_, "Fail to Create Slam Processor Instance, Abort!");

}

bool SlamRosWrapper::initProcessorParams() {
  slam_proc_->setgenerateMap(false);
  slam_proc_->setLinearUpdateDistanceThreshold(linear_u_dist_);
  slam_proc_->setAngularUpdateDistanceThreshold(angular_u_dist_);
  slam_proc_->setTemporalUpdatePeriodThreshold(-1.0);
  slam_proc_->setOSigmaGain(3.0);
  slam_proc_->setResampleThreshold(resample_th_);

  // Set scan matcher params

  ros::NodeHandle sm_nh_(pnh_, "scanmatch");
  // GMapping Original scanmatch
  double sm_sigma, sm_lstep, sm_astep, sm_lsigma, sm_min_score;
  int sm_kernel_size,sm_iter, sm_lskip;
  sm_nh_.param("sigma",sm_sigma, 0.05);
  sm_nh_.param("kernel_size", sm_kernel_size, 1);
  sm_nh_.param("lstep", sm_lstep, 0.05);
  sm_nh_.param("astep", sm_astep, 0.05);
  sm_nh_.param("iterations", sm_iter, 5);
  sm_nh_.param("lsigma", sm_lsigma, 0.075);
  sm_nh_.param("lskip",sm_lskip, 0);
  sm_nh_.param("minimum_score", sm_min_score, 0.0);

  slam_proc_->setMatchingParameters(laser_max_usable_range_, laser_max_range_,
  sm_sigma, sm_kernel_size, sm_lstep, sm_astep, sm_iter, sm_lsigma, (unsigned int)sm_lskip);
  slam_proc_->setMinimumMatchingScore(sm_min_score);
  //TODO -- Set All SLAM Params to cgr_slam Processor instance
  /*
  gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
  gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
  gsp_->setUpdatePeriod(temporalUpdate_);
  gsp_->setgenerateMap(false);
  gsp_->GridSlamProcessor::init(particles_, xmin_, ymin_, xmax_, ymax_,
                                delta_, initialPose);
  gsp_->setllsamplerange(llsamplerange_);
  gsp_->setllsamplestep(llsamplestep_);
  /// @todo Check these calls; in the gmapping gui, they use
  /// llsamplestep and llsamplerange intead of lasamplestep and
  /// lasamplerange.  It was probably a typo, but who knows.
  gsp_->setlasamplerange(lasamplerange_);
  gsp_->setlasamplestep(lasamplestep_);
  gsp_->setminimumScore(minimum_score_);

*/
  return true;
}

double SlamRosWrapper::computePoseEntropy() {
  double weight_total=0.0;
  for(std::vector<cgr_slam::CgrSlam2DProcessor::Particle>::const_iterator it = slam_proc_->getParticles().begin();
      it != slam_proc_->getParticles().end();
      ++it)
  {
    weight_total += it->weight_;
  }
  double entropy = 0.0;
  for(std::vector<cgr_slam::CgrSlam2DProcessor::Particle>::const_iterator it = slam_proc_->getParticles().begin();
      it != slam_proc_->getParticles().end();
      ++it)
  {
    if(it->weight_/weight_total > 0.0)
      entropy += it->weight_/weight_total * log(it->weight_/weight_total);
  }
  return -entropy;
}






