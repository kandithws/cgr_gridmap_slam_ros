//
// Created by kandithws on 6/6/2560.
//

#ifndef CGR_GRIDMAP_SLAM_ROS_CGR_SLAM2D_PROCESSOR_H
#define CGR_GRIDMAP_SLAM_ROS_CGR_SLAM2D_PROCESSOR_H

/*
 * @brief Occupancy Grid Map based Corrective Gradient Refinement for SLAM
 *
 *  original paper Joydeep Biswas "Corrective Gradient Refinement for Mobile Robot Localization." (IROS, 2011)
 */



// EIGEN MUST BE INCLUDED AFTER MRPT !!
// #include <Eigen/Geometry>
#include <gmapping/grid/map.h>
#include <gmapping/grid/harray2d.h>
#include <gmapping/utils/point.h>
#include <gmapping/scanmatcher/smmap.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/utils/stat.h>
#include <gmapping/scanmatcher/scanmatcher.h>
#include <gmapping/particlefilter/particlefilter.h>
#include <vector>
#include <deque>
#include <map>
#include "param_macros.h"
#include "motion_model.h"
#include "cgr_scan_matcher.h"



namespace cgr_slam {
// personal typedef for  short-hand typing
typedef GMapping::RangeSensor LaserScanSensor;
typedef GMapping::RangeReading Scan;
typedef GMapping::OrientedPoint Pose2D;
typedef GMapping::OdometrySensor OdomSensor;
typedef GMapping::Map<GMapping::PointAccumulator, GMapping::HierarchicalArray2D<GMapping::PointAccumulator > > HMap2D;


class CgrSlam2DProcessor{
   public:
    CgrSlam2DProcessor(int num_particles, double x_size_min, double x_size_max, double y_size_min, double y_size_max,
                       double resol, Pose2D init_pose=Pose2D(0,0,0));

    // Wrappers for Parameters settings
    void setLaserConfig(cgr_slam::LaserScanSensor* laser);
    bool setDiffDriveParams(double srr, double srt, double str, double stt);
    void setUpdateDistances(double linear, double angular);
    void resetAndBuildParticles();
    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
                                    int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0);
    /**
     *  @brief A main interface for CGR SLAM
     *  @param scan: latest scan in odom_frame(ROS) (predicted initial pose)
     * */
    bool processReading(Scan& scan);
    bool getParticleTrajectory(int particle_index);

    /**Copy from Gmapping: This class defines a particle of the filter.
 * Each particle has a map, a pose,
 * a weight and retains the current node in the trajectory tree*/

    /**
 * Copy From Gmapping, Node represent a vertex for trajectory tree*/

    class TrajectoryNode {
     public:
      /**Constructs a node of the trajectory tree.
         @param pose:      the pose of the robot in the trajectory
         @param weight:    the weight of the particle at that point in the trajectory
         @param accWeight: the cumulative weight of the particle
         @param parent:    the parent node in the tree
         @param childs:    the number of childs
        */
      TrajectoryNode(const Pose2D& pose, double weight, TrajectoryNode* parent=0, unsigned int childs=0);

      /**Destroys a tree node, and consistently updates the tree. If a node whose parent has only one child is deleted,
       also the parent node is deleted. This because the parent will not be reacheable anymore in the trajectory tree.*/
      ~TrajectoryNode();

      /**The pose of the robot*/
      Pose2D pose_;

      /**The weight of the particle*/
      double weight_;

      /**The sum of all the particle weights in the previous part of the trajectory*/
      double acc_weight_;

      double gweight_;


      /**The parent*/
      TrajectoryNode* parent_;

      /**The range reading to which this node is associated*/
      const Scan* reading_;

      /**The number of childs*/
      unsigned int childs_;

      /**counter in visiting the node (internally used)*/
      mutable unsigned int visit_counter_;

      /**visit flag (internally used)*/
      mutable bool flag_;
    };

    typedef std::vector<TrajectoryNode*> TrajectoryNodeVector;
    typedef std::deque<TrajectoryNode*> TrajectoryNodeDeque;
    typedef std::multimap<const TrajectoryNode*, TrajectoryNode*> TrajectoryNodeMultimap;

    class Particle {
      /**constructs a particle, given a map
     @param map: the particle map
      */
     public:
      Particle(const cgr_slam::HMap2D &map);
      // Override explicit casting operator i.e.: Particle p(..); double current_weight = double(p);
      /** @returns the weight of a particle */
      inline operator double() const { return weight_; }
      /** @returns the pose of a particle */
      inline operator Pose2D() const { return pose_; }
      /** sets the weight of a particle
      @param w the weight
      */
      inline void setWeight(double w) { weight_ = w; }

      inline TrajectoryNode *node() const { return node_; }
      /** The map */
      HMap2D map_;
      /** The pose of the robot */
      Pose2D pose_;

      /** The pose of the robot at the previous time frame (used for computing thr odometry displacements) */
      Pose2D previous_pose_;

      /** The weight of the particle */
      double weight_;

      /** The cumulative weight of the particle */
      double weight_sum_;

      double gweight_;

      /** The index of the previous particle in the trajectory tree */
      int previous_index_;

      /** Entry to the trajectory tree */
      TrajectoryNode* node_;
    };

    typedef std::vector<Particle> ParticleVector;

    inline const ParticleVector& getParticles() const {return particles_; }

    inline const std::vector<unsigned int>& getIndexes() const{return indexes_; }
    int getBestParticleIndex() const;

    MotionModel motion_model_;

   private:
    //void copyFirstStageProposal();
    void performRefineAndAccept(const double* plainReading);
    void performUpdate();
    void performResample();
    void scanMatch(const double* plainReading);
    bool resample(const double* plainReading, int adaptSize, const Scan* reading);
    void normalize();

    // TREE ALGORITHMS
    TrajectoryNodeVector getTrajectories() const;
    void updateTreeWeights(bool weightsAlreadyNormalized);
    void integrateScanSequence(TrajectoryNode* node);
    void resetTree();
    double propagateWeight(TrajectoryNode* n, double weight);
    double propagateWeights();
    /**the scanmatcher algorithm*/
    CgrScanMatcher matcher_;

    /*Private Use variables*/
    bool is_first_scan_received_ = false;
    double last_update_time_;
    double acc_linear_distance_;
    double acc_angular_distance_;
    ParticleVector particles_;
    /**the particle indexes after resampling (internally used)*/
    std::vector<unsigned int> indexes_;
    /**the particle weights (internally used)*/
    std::vector<double> weights_;

    /*CGR Refine step poses Cache, for debug*/
    std::vector<Pose2D> q0;
    std::vector<double> q0_lik;
    std::vector<Pose2D> qr;
    std::vector<double > qr_lik;

    Pose2D odom_pose_;
    Pose2D last_part_pose_;
    double m_neff;
    // Params

    Pose2D initial_pose_;
    //unsigned int num_laser_beams_;

    CGR_PARAM(MotionModelType, motion_model_type, std::string, "diff_drive")
    CGR_PARAM(NumParticles, num_particles, int, 30)
    CGR_PARAM(XSizeMin, x_size_min, double, 0)
    CGR_PARAM(XSizeMax, x_size_max, double, 0)
    CGR_PARAM(YSizeMin, y_size_min, double, 0)
    CGR_PARAM(YSizeMax, y_size_max, double, 0)
    CGR_PARAM(MapResolution, resol, double, 0.05)
    CGR_PARAM(NumLaserBeams, num_laser_beams, unsigned int, 180)
    CGR_PARAM(LinearUpdateDistanceThreshold, linear_update_dist_th, double, 0.5)
    CGR_PARAM(AngularUpdateDistanceThreshold, angular_update_dist_th, double, 0.5)
    CGR_PARAM(TemporalUpdatePeriodThreshold, temporal_update_time_th, double, -1.0)
    CGR_PARAM(ResampleThreshold, resample_th, double, 0.5)
    CGR_PARAM(DistanceThresholdCheck, dist_th_check, double, 20.0) // Max move per frame
    CGR_PARAM(OSigmaGain, o_sigma_gain, double, 3.0)
    CGR_PARAM(MinimumMatchingScore, minimum_score, double, 50.0)
    CGR_PARAM(UseGmapping, use_gmapping, bool, false)
    CGR_PARAM(MaxIcpIteration, max_icp_iter, int, 5)
    // Use the original version of Motion Model of Odometry Model (from Probabilistic Robotics)
    // Or use EKFLinearlized version (Gmapping) instead
    CGR_PARAM(UseTrueDiffDriveMotionModel, true_diff_drive_motion_model, bool, false)
    CGR_MEMBER_VAR_PARAM(matcher_, generateMap, bool)

  };



}


#endif //CGR_GRIDMAP_SLAM_ROS_CGR_SLAM2D_PROCESSOR_H
