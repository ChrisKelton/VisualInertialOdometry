#include <math.h>
#include <iostream>

#include "GraphSolver.h"
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <sstream>


void GraphSolver::addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation) {
  // Request access to the imu measurements
  std::unique_lock<std::mutex> lock(imu_mutex);

  // Append this new measurement to the array
  imu_times.push_back(timestamp);
  imu_linaccs.push_back(linacc);
  imu_angvel.push_back(angvel);
  imu_orientation.push_back(orientation);

}

void GraphSolver::addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv, const rclcpp::Logger& logger) {

  // Return if the node already exists in the graph
  if (ct_state_lookup.find(timestamp) != ct_state_lookup.end()) {
    printf("[UV] duplicate timestamp %.3f — skipping\n", timestamp);
    return;
  }

  // Return if we don't actually have any plane measurements
  if(leftids.empty()) {
    printf("[UV] no features — skipping\n");
    return;
  }

  // Return if we don't actually have any IMU measurements
  if(imu_times.size() < 2) {
    printf("[UV] imu_times.size()=%zu < 2 — skipping (init=%d ct=%zu)\n",
           imu_times.size(), (int)systeminitalized, ct_state);
    return;
  }

  // We should try to initialize now
  // Or add the current a new IMU measurement and state!
  if(!systeminitalized) {

      RCLCPP_INFO(logger, "Initializing system");
      initialize(timestamp, logger);

      // Return if we have not initialized the system yet
      if(!systeminitalized) {
        RCLCPP_INFO(logger, "Failed to initialize system");
        return;
      }

  } else {

      // The first 2 imuFactors will be exclusively dead-reckoning b/c we are not optimizing until we have 2 image
      // features. (Need at least 2 points for gtsam::Triangulation)
      // Forster2 discrete preintegration
      gtsam::CombinedImuFactor imuFactor = create_imu_factor(timestamp);
      graph_new->add(imuFactor);
      graph->add(imuFactor);
      // // Integrate IMU measurements for state prediction (skip CombinedImuFactor
      // // construction — crashes with GTSAM 4.2.1 + TBB 2021 on this system).
      // integrate_imu(timestamp);

      // Original models
      gtsam::State newstate = get_predicted_state(values_initial);

      // Move node count forward in time
      ct_state++;

      // These are the variables in the factor graph (poses)
      // The factors are the preintegrated IMU factors & landmark measurements
      // Append to our node vectors
      values_new.insert(    X(ct_state), newstate.pose());
      values_new.insert(    V(ct_state), newstate.v());
      values_new.insert(    B(ct_state), newstate.b());
      values_initial.insert(X(ct_state), newstate.pose());
      values_initial.insert(V(ct_state), newstate.v());
      values_initial.insert(B(ct_state), newstate.b());

      // Add ct state to map
      ct_state_lookup[timestamp] = ct_state;
      timestamp_lookup[ct_state] = timestamp;
  }

  // Assert our vectors are equal (note will need to remove top one eventually)
  assert(leftids.size() == leftuv.size());

  // Request access
  std::unique_lock<std::mutex> features_lock(features_mutex);

  // If we are using inverse depth, then lets call on it
  process_feat_smart(timestamp, leftids, leftuv, logger);
  // // Smart feature factors — disabled to isolate GTSAM SIGSEGV; IMU-only for now
  // // process_feat_smart(timestamp, leftids, leftuv);
}

void GraphSolver::optimize(const rclcpp::Logger& logger) {

  // Return if not initialized
  if(!systeminitalized && ct_state < 2) {
	  // Wait until we have added more than 1 frame, otherwise we will get all degenerate points in our factor graph.
	  RCLCPP_INFO(logger, "[OPT] system not initialized AND ct_state < 2");
      return;
  }
  if (systeminitalized && ct_state < 1) {
      // Wait until we have added more than 1 frame, otherwise we will get all degenerate points in our factor graph.
      RCLCPP_INFO(logger, "[OPT] system initialized BUT ct_state < 1");
      return;
  }

  // Defer ISAM2 until the init window is full.  During this period new states
  // are dead-reckoned by IMU preintegration and SmartFactor observations
  // accumulate in measurement_smart_lookup_left; per-update staging buffers
  // are cleared and IMU preintegration is reset so each state starts fresh.
  if (ct_state < (size_t)config->initWindow) {
      RCLCPP_INFO(logger, "[OPT] Init window: %zu / %d — deferring ISAM2",
                  ct_state, config->initWindow);
      smart_factors_to_remove_.clear();
      smart_factor_graph_new_entries_.clear();
      values_new.clear();
      graph_new->resize(0);
      reset_imu_integration();
      return;
  }

  // First real ISAM2 update: build a full batch graph from all accumulated
  // priors + IMU factors (in *graph) plus every SmartFactor with >= 3
  // observations (latest version from measurement_smart_lookup_left).
  // A brief LM pass refines the dead-reckoned linearization point before
  // handing off to incremental ISAM2.
  if (!isam2_initialized_) {
      isam2_initialized_ = true;

      // Rebuild graph_new as the complete batch: copy priors + all IMU
      // factors from *graph, then append SmartFactors.
      graph_new->resize(0);
      for (size_t i = 0; i < graph->size(); ++i)
          graph_new->push_back(graph->at(i));

      smart_factors_to_remove_.clear();
      smart_factor_graph_new_entries_.clear();
      for (auto& [feat_id, factor] : measurement_smart_lookup_left) {
          if (factor->keys().size() >= 3) {
              smart_factor_graph_new_entries_.emplace_back(feat_id, graph_new->size());
              graph_new->push_back(factor);
          }
      }

      // Rebuild values_new: every state accumulated during the window is
      // new to ISAM2 (none have been inserted yet).
      values_new.clear();
      for (size_t k = 0; k <= ct_state; ++k) {
          values_new.insert(X(k), values_initial.at<gtsam::Pose3>(X(k)));
          values_new.insert(V(k), values_initial.at<gtsam::Vector3>(V(k)));
          values_new.insert(B(k), values_initial.at<gtsam::Bias>(B(k)));
      }

      // Brief LM pass to sharpen the linearization point before ISAM2.
      RCLCPP_INFO(logger, "[OPT] Init window full (%zu states, %zu SmartFactors) — running LM batch init",
                  ct_state + 1, smart_factor_graph_new_entries_.size());
      try {
          gtsam::LevenbergMarquardtParams lm_params;
          lm_params.maxIterations = 20;
          gtsam::LevenbergMarquardtOptimizer lm(*graph_new, values_initial, lm_params);
          gtsam::Values lm_result = lm.optimize();
          bool complete = true;
          for (size_t k = 0; k <= ct_state && complete; ++k)
              complete = lm_result.exists(X(k)) && lm_result.exists(V(k)) && lm_result.exists(B(k));
          if (complete) {
              values_initial = lm_result;
              values_new.clear();
              for (size_t k = 0; k <= ct_state; ++k) {
                  values_new.insert(X(k), values_initial.at<gtsam::Pose3>(X(k)));
                  values_new.insert(V(k), values_initial.at<gtsam::Vector3>(V(k)));
                  values_new.insert(B(k), values_initial.at<gtsam::Bias>(B(k)));
              }
              RCLCPP_INFO(logger, "[OPT] LM batch init converged (error=%.3f)", lm.error());
          } else {
              RCLCPP_WARN(logger, "[OPT] LM batch init produced incomplete values — using dead-reckoned initial values");
          }
      } catch (const std::exception& e) {
          RCLCPP_WARN(logger, "[OPT] LM batch init failed (%s) — using dead-reckoned initial values", e.what());
      }
  }

  RCLCPP_INFO(logger, "[OPT] ct_state=%zu  graph_new=%zu  values_new=%zu  to_remove=%zu",
              ct_state, graph_new->size(), values_new.size(), smart_factors_to_remove_.size());

  // Perform smoothing update
  try {
    // Superseded by the init-window LM batch above (dead code when initWindow >= 2).
    // if (ct_state == 1) {
    //     RCLCPP_INFO(logger, "[OPT] Optimizing state 1 fully.");
    //     auto batchOptimizer = gtsam::LevenbergMarquardtOptimizer(*graph_new, values_new);
    //     gtsam::Pose3 pose1_unopt = values_new.at<gtsam::Pose3>(X(1));
    //     Eigen::Vector3d vel1_unopt = values_new.at<gtsam::Vector3>(V(1));
    //     gtsam::imuBias::ConstantBias bias1_unopt = values_new.at<gtsam::Bias>(B(1));
    //     gtsam::State state1(pose1_unopt, vel1_unopt, bias1_unopt);
    //     values_new = batchOptimizer.optimize();
    //     prev_state = gtsam::State(
    //         values_new.at<gtsam::Pose3>(X(1)),
    //         values_new.at<Eigen::Vector3d>(V(1)),
    //         values_new.at<gtsam::imuBias::ConstantBias>(B(1))
    //     );
    //     gtsam::State state_del = state1 - prev_state;
    //     std::ostringstream oss;
    //     oss << state_del;
    //     RCLCPP_INFO(logger, "[OPT] Finished full optimization. Change in 1st state   %s", oss.str().c_str());
    // }

    gtsam::ISAM2Result result = isam2->update(*graph_new, values_new, smart_factors_to_remove_);

    // Record the new ISAM2 factor index for each SmartFactor that was added or
    // replaced this cycle; result.newFactorsIndices[i] corresponds to graph_new[i].
    for (auto& [feat_id, graph_pos] : smart_factor_graph_new_entries_)
      smart_factor_isam_index_left_[feat_id] = result.newFactorsIndices[graph_pos];

    values_initial = isam2->calculateEstimate();
    const gtsam::State latest_state(
      values_initial.at<gtsam::Pose3>(X(ct_state)),
      values_initial.at<Eigen::Vector3d>(V(ct_state)),
      values_initial.at<gtsam::imuBias::ConstantBias>(B(ct_state))
    );
    std::ostringstream oss;
    oss << latest_state;
    RCLCPP_INFO(logger, "[OPT] ISAM2 OK   %s", oss.str().c_str());
    oss.str("");
    oss.clear();
    gtsam::State state_diff = prev_state - latest_state;
    oss << state_diff;
    RCLCPP_INFO(logger, "[OPT] State update for latest state '%lu':   %s", ct_state, oss.str().c_str());
    // const auto& p = values_initial.at<gtsam::Pose3>(X(ct_state)).translation();
    // RCLCPP_INFO(logger, "[OPT] ISAM2 OK  pos=(%.3f, %.3f, %.3f)", p(0), p(1), p(2));
    prev_state = latest_state;

    // target stream
    std::stringstream result_stream;
    // Save the original cout buffer and swap it with the new stream's buffer
    std::streambuf* original_buffer = std::cout.rdbuf(result_stream.rdbuf());
    result.print();
    // Restore the original buffer before result_stream is destroyed
    std::cout.rdbuf(original_buffer);
    RCLCPP_INFO(logger, "[OPT - result] %s", result_stream.str().c_str());
    gtsam::NonlinearFactorGraph nonlinearGraph = isam2->getFactorsUnsafe();
    RCLCPP_INFO(logger, "[OPT - graph] Number of factors & variables: '%zu' & '%zu'", nonlinearGraph.size(), values_initial.size());

    size_t non_degenerate = 0;
    for (auto& [id, f] : measurement_smart_lookup_left)
      if (f->point(values_initial)) non_degenerate++;
    RCLCPP_INFO(logger, "[OPT] SmartFactors: %zu / %zu non-degenerate",
                non_degenerate, measurement_smart_lookup_left.size());

    bool verbose = true;
    bool extra_verbose = false;
    if (non_degenerate != measurement_smart_lookup_left.size() && verbose) {
	  // Linearize the graph into a Gaussian factor graph
	  gtsam::GaussianFactorGraph linearGraph = *nonlinearGraph.linearize(values_initial);
	  // Convert to a raw sparse matrix configuration
	  std::pair<gtsam::Matrix, gtsam::Vector> system = linearGraph.jacobian();
	  gtsam::Matrix A = system.first;
	  // Perform SVD Decomposition to inspect singular values
	  Eigen::JacobiSVD<gtsam::Matrix> svd(A);
	  std::stringstream ss;
	  ss << svd.singularValues().transpose();
	  RCLCPP_INFO(logger, "[OPT - Degeneracy] Singular values: %s", ss.str().c_str());

	  ss.str("");
	  ss.clear();
      if (extra_verbose) {
	    for (size_t i = 0; i < linearGraph.size(); ++i) {
		  auto factor = linearGraph.at(i);
		  if (!factor) continue;  // Skip deleted slots in iSAM2 arrays

		  RCLCPP_INFO(logger, "[OPT] Factor index: %zu involves keys: ", i);
		  for (gtsam::Key key : factor->keys()) {
		    ss << gtsam::DefaultKeyFormatter(key);
		    ss << " ";
		  }
		  RCLCPP_INFO(logger, "\t%s\n", ss.str().c_str());
		  ss.str("");
		  ss.clear();
		  std::streambuf* oldCoutBuffer = std::cout.rdbuf(ss.rdbuf());
		  factor->print("Factor Details: ");
		  std::cout.rdbuf(oldCoutBuffer);
		  RCLCPP_INFO(logger, "\t%s", ss.str().c_str());
		  ss.str("");
		  ss.clear();
	    }
      }
	}
  } catch (gtsam::IndeterminantLinearSystemException& e) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam indeterminate linear system — rebuilding ISAM2: %s", e.what());
    rebuild_isam2(logger);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam | Runtime error — skipping update: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam | Standard error — skipping update: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam | Unknown error — skipping update");
  }

  smart_factors_to_remove_.clear();
  smart_factor_graph_new_entries_.clear();

  // // GTSAM ISAM2 and LM both crash on this system (GTSAM 4.2.1 + TBB 2021
  // // concurrent_unordered_map allocator incompatibility).
  // // IMU dead-reckoning: states are already inserted into values_initial by
  // // addmeasurement_uv via get_predicted_state(); just clear the buffers.

  // Remove the used up nodes
  values_new.clear();

  // Remove the used up factors
  graph_new->resize(0);

  // reset imu preintegration
  reset_imu_integration();
}

void GraphSolver::rebuild_isam2(const rclcpp::Logger& logger) {
  RCLCPP_WARN(logger, "[rebuild_isam2]: Rebuilding ISAM2 (%zu imu/prior factors, %zu smart factors tracked)",
              graph->size(), measurement_smart_lookup_left.size());

  delete isam2;
  gtsam::ISAM2Params isam_params;
  isam_params.relinearizeThreshold = 0.01;
  isam_params.relinearizeSkip = 1;
  isam_params.cacheLinearizedFactors = false;
  isam_params.enableDetailedResults = true;
  isam2 = new gtsam::ISAM2(isam_params);
  smart_factor_isam_index_left_.clear();

  // Non-smart factors (priors + CombinedImuFactors) first, then multi-observation SmartFactors.
  gtsam::NonlinearFactorGraph rebuild_graph = *graph;
  std::vector<int> rebuild_feat_ids;
  for (auto& [feat_id, factor] : measurement_smart_lookup_left) {
    if (factor->keys().size() >= 2) {
      rebuild_feat_ids.push_back(feat_id);
      rebuild_graph.push_back(factor);
    }
  }

  try {
    gtsam::ISAM2Result result = isam2->update(rebuild_graph, values_initial);
    const size_t base = graph->size();
    for (size_t i = 0; i < rebuild_feat_ids.size(); i++)
      smart_factor_isam_index_left_[rebuild_feat_ids[i]] = result.newFactorsIndices[base + i];
    values_initial = isam2->calculateEstimate();
    RCLCPP_INFO(logger, "[rebuild_isam2]: Rebuilt with %zu total factors", rebuild_graph.size());
  } catch (const std::exception& ex) {
    // Last resort: drop all SmartFactors and keep IMU/prior chain only.
    RCLCPP_ERROR(logger, "[rebuild_isam2]: Rebuild failed (%s) — clearing SmartFactors, IMU-only fallback", ex.what());
    delete isam2;
    isam2 = new gtsam::ISAM2(isam_params);
    smart_factor_isam_index_left_.clear();
    measurement_smart_lookup_left.clear();
    isam2->update(*graph, values_initial);
    values_initial = isam2->calculateEstimate();
  }
}

void GraphSolver::initialize(double timestamp, const rclcpp::Logger& logger) {

  // If we have already initialized, then just return
  if (systeminitalized) {
    return;
  }

  // Wait for enough IMU readings if we initialize from rest
  if (imu_times.size() < config->imuWait) {
  	RCLCPP_INFO(logger, "[initialize]: Waiting for additional IMU readings: %zu / %i", imu_times.size(), config->imuWait);
    return;
  }

  // Reading from launch file
  gtsam::Vector4 q_GtoI = config->prior_qGtoI;
  gtsam::Vector3 p_IinG = config->prior_pIinG;
  gtsam::Vector3 v_IinG = config->prior_vIinG;
  gtsam::Vector3 ba = config->prior_ba;
  gtsam::Vector3 bg = config->prior_bg;

  // Create prior factor and add it to the graph
  gtsam::State prior_state = gtsam::State(gtsam::Pose3(gtsam::Quaternion(q_GtoI(3), q_GtoI(0), q_GtoI(1), q_GtoI(2)), p_IinG),
                                          v_IinG, gtsam::Bias(ba, bg)); // gtsam::Quaternion(w, x, y, z)
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(config->sigma_prior_rotation),
                                                         gtsam::Vector3::Constant(config->sigma_prior_translation)).finished());
  auto v_noise = gtsam::noiseModel::Isotropic::Sigma(3, config->sigma_velocity);
  auto b_noise = gtsam::noiseModel::Isotropic::Sigma(6, config->sigma_bias);

  graph_new->add(gtsam::PriorFactor<gtsam::Pose3>(  X(ct_state), prior_state.pose(), pose_noise));
  graph_new->add(gtsam::PriorFactor<gtsam::Vector3>(V(ct_state), prior_state.v(),    v_noise));
  graph_new->add(gtsam::PriorFactor<gtsam::Bias>(   B(ct_state), prior_state.b(),    b_noise));
  graph->add(gtsam::PriorFactor<gtsam::Pose3>(      X(ct_state), prior_state.pose(), pose_noise));
  graph->add(gtsam::PriorFactor<gtsam::Vector3>(    V(ct_state), prior_state.v(),    v_noise));
  graph->add(gtsam::PriorFactor<gtsam::Bias>(       B(ct_state), prior_state.b(),    b_noise));

  // Add initial state to the graph
  values_new.insert(    X(ct_state), prior_state.pose());
  values_new.insert(    V(ct_state), prior_state.v());
  values_new.insert(    B(ct_state), prior_state.b());
  values_initial.insert(X(ct_state), prior_state.pose());
  values_initial.insert(V(ct_state), prior_state.v());
  values_initial.insert(B(ct_state), prior_state.b());

  // Add ct state to map
  ct_state_lookup[timestamp] = ct_state;
  timestamp_lookup[ct_state] = timestamp;

  // Clear all old imu messages (keep the last two)
  // Need to clear the old messages, b/c we don't know when the previous messages were exactly. They could be very
  // very far away from our new keyframe. Thus, being far away from the next keyframe.
  imu_times.erase(imu_times.begin(), imu_times.end() - 1);
  imu_linaccs.erase(imu_linaccs.begin(), imu_linaccs.end() - 1);
  imu_angvel.erase(imu_angvel.begin(), imu_angvel.end() - 1);
  imu_orientation.erase(imu_orientation.begin(), imu_orientation.end() - 1);

  if (set_imu_preintegration(prior_state, logger)) {
    RCLCPP_INFO(logger, "[initialize]: Initialized system successfully!");
    systeminitalized = true;
  } else {
    RCLCPP_INFO(logger, "[initialize]: Failed to initialize system! :(");
  }

  printf("[INIT]: system initialized at t=%.3f\n", timestamp);
  printf("[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\n",q_GtoI(0),q_GtoI(1),q_GtoI(2),q_GtoI(3));
  printf("[INIT]: velocity = %.4f, %.4f, %.4f\n",v_IinG(0),v_IinG(1),v_IinG(2));
  printf("[INIT]: position = %.4f, %.4f, %.4f\n",p_IinG(0),p_IinG(1),p_IinG(2));
  printf("[INIT]: bias accel = %.4f, %.4f, %.4f\n",ba(0),ba(1),ba(2));
  printf("[INIT]: bias gyro = %.4f, %.4f, %.4f\n",bg(0),bg(1),bg(2));
}
