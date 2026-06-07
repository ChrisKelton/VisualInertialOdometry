#pragma once

#include <mutex>
#include <thread>
#include <deque>
#include <unordered_map>

// Graphs
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

// Factors
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>

#include <rclcpp/rclcpp.hpp>

#include "utils/Config.h"
#include "utils/State.h"

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

typedef std::vector<std::pair<double, gtsam::State>> Trajectory;
typedef gtsam::SmartProjectionPoseFactor<gtsam::Cal3_S2> SmartFactor;

class GraphSolver {
public:
  GraphSolver(Config* config, bool publish_landmark_covariances = false) {
    
    // Set up config
    this->config = config;
    this->graph = new gtsam::NonlinearFactorGraph();
    this->graph_new = new gtsam::NonlinearFactorGraph();
    
    // ISAM2 solver
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_params.cacheLinearizedFactors = false;
    isam_params.enableDetailedResults = true;

    // // ISAM2 Dogleg parameters (adds damping to prevent indeterminate linear system crashes)
    // gtsam::ISAM2DoglegParams dogleg_params;
    // // Adjust initial trust region radius if needed (default is usually 1.0)
    // dogleg_params.initialDelta = 1.0;
    // // Assign dogleg parameters to the main solver parameters
    // isam_params.optimizationParams = dogleg_params;

    isam_params.print();
    this->isam2 = new gtsam::ISAM2(isam_params);


    // Flag to publish landmark covariances
    this->publish_landmark_covariances = publish_landmark_covariances;
  }

  /// Will return true if the system is initialized
  inline bool is_initialized() { return systeminitalized; }

  /// Function that takes in IMU measurements for use in preintegration measurements
  void addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation);

  /// Function that takes in UV measurements that will be used as "features" in our graph
  void addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv, const rclcpp::Logger& logger);

  /// This function will optimize the graph
  void optimize(const rclcpp::Logger& logger);
  
  /// This function returns the current state, return origin if we have not initialized yet
  gtsam::State get_state(size_t ct) {
    // Ensure valid states
    assert (ct <= ct_state);
    if (!values_initial.exists(X(ct)))
      return gtsam::State();
    return gtsam::State(values_initial.at<gtsam::Pose3>(  X(ct)),
                 values_initial.at<gtsam::Vector3>(V(ct)),
                 values_initial.at<gtsam::Bias>(   B(ct)));
  }

  gtsam::State get_current_state() { 
    return get_state(ct_state);
  }

  Trajectory get_trajectory(const rclcpp::Logger& logger) {
    // Return if we do not have any nodes yet
    if (values_initial.empty()) {
      RCLCPP_INFO(logger, "'values_initial' is empty. Returning empty trajectory...");
      Trajectory traj;
      traj.push_back(std::make_pair(0.0, gtsam::State()));
      return traj;
    }
    Trajectory trajectory;
    // Else loop through the states and return them
    for (size_t i = 1; i <= ct_state; ++i) {
      double timestamp = timestamp_lookup[i];
      trajectory.push_back(std::make_pair(timestamp, get_state(i)));
    }
    return trajectory;
  }
  
  /// Returns the currently tracked features
  std::vector<Eigen::Vector3d> get_current_features() {
      if(values_initial.empty()) {
          return std::vector<Eigen::Vector3d>();
      }
      std::vector<Eigen::Vector3d> features;
      for (auto& [id, f] : measurement_smart_lookup_left) {
          // Skip factors not yet inserted into ISAM2 — pending factors (< 3
          // observations) triangulate from dead-reckoned poses and produce
          // garbage 3D points that appear to jump far from the trajectory.
          if (!smart_factor_isam_index_left_.count(id)) continue;
          gtsam::TriangulationResult point = f->point(values_initial);
          if (point)
              features.push_back(*point);
      }
      return features;
  }

  // std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> get_current_features() {
  //   // Return if we do not have any nodes yet
  //   if(values_initial.empty()) {
  //     return std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>>();
  //   }
  //   // Our vector of points in the global
  //   std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> features_and_covariances;
  //   // Else loop through the features and return them
  //   for (auto element : measurement_smart_lookup_left) {
  //     // gtsam::TriangulationResult tr_point = element.second->point(values_initial);
  //     auto factor = element.second;
  //     // Collect the keys of the poses observing this landmark
  //     gtsam::KeyVector poseKeys = factor->keys();
  //     // For a full joint covariance, extract the blocks or request them individually
  //     gtsam::Matrix cov_pose_i = isam2->marginalCovariance(poseKeys[0]);
  //     gtsam::TriangulationResult tr_point = factor->point(values_initial);
  //     if (tr_point.valid()) {
  //       gtsam::Point3 point = *tr_point;
  //       Eigen::Matrix3d covariance;
  //       covariance = factor->pointCov
  //       features_and_covariances.push_back(std::make_pair(point, covariance));
  //     }
  //   }
  //   return features_and_covariances;
  // }

  std::deque<double> get_imu_times() {
    return imu_times;
  }

private:
  /// Functions
  // Function which will try to initalize our graph using the current IMU measurements
  void initialize(double timestamp, const rclcpp::Logger& logger);

  // ******************************* TODO ********************************* //
  bool set_imu_preintegration(const gtsam::State& prior_state, const rclcpp::Logger& logger);
  // void integrate_imu(double updatetime);

  // Function that will compound the GTSAM preintegrator to get discrete preintegration measurement
  gtsam::CombinedImuFactor create_imu_factor(double updatetime);

  // Function will get the predicted Navigation State based on this generated measurement 
  gtsam::State get_predicted_state(gtsam::Values& values_initial);

  // Function that will reset imu preintegration
  void reset_imu_integration();

  // Smart feature measurements
  void process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv, const rclcpp::Logger& logger);

  // Rebuild ISAM2 from the full factor graph after a failed update
  void rebuild_isam2(const rclcpp::Logger& logger);
  // ******************************* END TODO ********************************* //

  // TODO: Implement
  void get_covariance_of_landmarks() {};

  /// Members
  /// Config object (has all sensor noise values)
  Config* config;

  // New factors that have not been optimized yet
  gtsam::NonlinearFactorGraph* graph_new;
  
  // Master non-linear GTSAM graph, all created factors
  gtsam::NonlinearFactorGraph* graph;

  // Last estimated state
  gtsam::State prev_state;

  // New nodes that have not been optimized
  gtsam::Values values_new;
  
  // All created nodes
  gtsam::Values values_initial;

  // ISAM2 solvers
  gtsam::ISAM2* isam2;
  bool publish_landmark_covariances = false;

  // Current ID of state and features
  size_t ct_state = 0;
  size_t ct_features = 0;

  /// Boolean that tracks if we have initialized
  bool systeminitalized = false;

  /// True once the init window is full and ISAM2 has received its first batch update.
  bool isam2_initialized_ = false;

  std::unordered_map<double, size_t> ct_state_lookup; // ct state based on timestamp
  std::unordered_map<size_t, double> timestamp_lookup;

  // IMU data from the sensor
  std::mutex imu_mutex;
  std::deque<double> imu_times;
  std::deque<Eigen::Vector3d> imu_linaccs;
  std::deque<Eigen::Vector3d> imu_angvel;
  std::deque<Eigen::Vector4d> imu_orientation;

  // Imu Preintegration
  gtsam::PreintegratedCombinedMeasurements* preint_gtsam = nullptr;

  /// Lookup tables for features
  std::mutex features_mutex;
  std::unordered_map<int, SmartFactor::shared_ptr> measurement_smart_lookup_left;

  // Maps feature ID to the ISAM2 factor index of its SmartFactor.
  // Required so we can remove-and-re-add the factor each time it gains a new
  // observation; without this, ISAM2's variableIndex_ becomes inconsistent with
  // the factor's actual key set, corrupting the Bayes tree and causing SIGSEGV.
  std::unordered_map<int, size_t> smart_factor_isam_index_left_;

  // Populated by process_feat_smart, consumed by optimize() each cycle.
  gtsam::FactorIndices smart_factors_to_remove_;
  std::vector<std::pair<int, size_t>> smart_factor_graph_new_entries_;  // (feature_id, position in graph_new)
};
