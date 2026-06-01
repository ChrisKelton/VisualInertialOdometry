#include <math.h>
#include <iostream>

#include "GraphSolver.h"


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
  process_feat_smart(timestamp, leftids, leftuv);
  // // Smart feature factors — disabled to isolate GTSAM SIGSEGV; IMU-only for now
  // // process_feat_smart(timestamp, leftids, leftuv);
}

void GraphSolver::optimize(const rclcpp::Logger& logger) {

  // Return if not initialized
  if(!systeminitalized && ct_state < 2)
      return;

  RCLCPP_INFO(logger, "[OPT] ct_state=%zu  graph_new=%zu  values_new=%zu  to_remove=%zu",
    ct_state, graph_new->size(), values_new.size(), smart_factors_to_remove_.size());

  // Perform smoothing update
  try {
    gtsam::ISAM2Result result = isam2->update(*graph_new, values_new, smart_factors_to_remove_);
    RCLCPP_INFO(logger, "[OPT] received ISAM2 result!");

    // Record the new ISAM2 factor index for each SmartFactor that was added or
    // replaced this cycle; result.newFactorsIndices[i] corresponds to graph_new[i].
    for (auto& [feat_id, graph_pos] : smart_factor_graph_new_entries_)
      smart_factor_isam_index_left_[feat_id] = result.newFactorsIndices[graph_pos];

    values_initial = isam2->calculateEstimate();
    const auto& p = values_initial.at<gtsam::Pose3>(X(ct_state)).translation();
    RCLCPP_INFO(logger, "[OPT] ISAM2 OK  pos=(%.3f, %.3f, %.3f)", p(0), p(1), p(2));
  } catch (gtsam::IndeterminantLinearSystemException& e) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam indeterminate linear system — skipping update: %s", e.what());
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam | Runtime error — skipping update: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam | Standard error — skipping update: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(logger, "FORSTER2 gtsam | Unknown error — skipping update");
  }

  RCLCPP_INFO(logger, "[OPT] Clearing smart factors...");
  smart_factors_to_remove_.clear();
  smart_factor_graph_new_entries_.clear();
  RCLCPP_INFO(logger, "[OPT] Cleared smart factors!");

  // // GTSAM ISAM2 and LM both crash on this system (GTSAM 4.2.1 + TBB 2021
  // // concurrent_unordered_map allocator incompatibility).
  // // IMU dead-reckoning: states are already inserted into values_initial by
  // // addmeasurement_uv via get_predicted_state(); just clear the buffers.

  // Remove the used up nodes
  values_new.clear();
  RCLCPP_INFO(logger, "[OPT] Cleared 'values_new'!");

  // Remove the used up factors
  graph_new->resize(0);
  RCLCPP_INFO(logger, "[OPT] Removed used factors from 'graph_new'!");

  // reset imu preintegration
  reset_imu_integration();
  RCLCPP_INFO(logger, "[OPT] Reset IMU Integration!");
}

void GraphSolver::initialize(double timestamp, const rclcpp::Logger& logger) {
  RCLCPP_INFO(logger, "[initialize]: Beginning...");

  // If we have already initialized, then just return
  if (systeminitalized) {
    RCLCPP_INFO(logger, "[initialize]: System already initialized!");
    return;
  }

  // Wait for enough IMU readings if we initialize from rest
  if (imu_times.size() < config->imuWait) {
    RCLCPP_INFO(logger, "[initialize]: Waiting for additional IMU readings: %zu / %i", imu_times.size(), config->imuWait);
    return;
  }

  RCLCPP_INFO(logger, "[initialize]: Reading from launch file...");
  // Reading from launch file
  gtsam::Vector4 q_GtoI = config->prior_qGtoI;
  gtsam::Vector3 p_IinG = config->prior_pIinG;
  gtsam::Vector3 v_IinG = config->prior_vIinG;
  gtsam::Vector3 ba = config->prior_ba;
  gtsam::Vector3 bg = config->prior_bg;
  RCLCPP_INFO(logger, "[initialize]: Read from launch file!");

  // Create prior factor and add it to the graph
  gtsam::State prior_state = gtsam::State(gtsam::Pose3(gtsam::Quaternion(q_GtoI(3), q_GtoI(0), q_GtoI(1), q_GtoI(2)), p_IinG),
                                          v_IinG, gtsam::Bias(ba, bg)); // gtsam::Quaternion(w, x, y, z)
  RCLCPP_INFO(logger, "[initialize]: Created prior_state!");
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(config->sigma_prior_rotation),
                                                         gtsam::Vector3::Constant(config->sigma_prior_translation)).finished());
  RCLCPP_INFO(logger, "[initialize]: Created pose_noise!");
  auto v_noise = gtsam::noiseModel::Isotropic::Sigma(3, config->sigma_velocity);
  auto b_noise = gtsam::noiseModel::Isotropic::Sigma(6, config->sigma_bias);
  RCLCPP_INFO(logger, "[initialize]: Created v_noise & b_noise!");

  graph_new->add(gtsam::PriorFactor<gtsam::Pose3>(  X(ct_state), prior_state.pose(), pose_noise));
  graph_new->add(gtsam::PriorFactor<gtsam::Vector3>(V(ct_state), prior_state.v(),    v_noise));
  graph_new->add(gtsam::PriorFactor<gtsam::Bias>(   B(ct_state), prior_state.b(),    b_noise));
  RCLCPP_INFO(logger, "[initialize]: Added nodes to 'graph_new'!");
  graph->add(gtsam::PriorFactor<gtsam::Pose3>(      X(ct_state), prior_state.pose(), pose_noise));
  graph->add(gtsam::PriorFactor<gtsam::Vector3>(    V(ct_state), prior_state.v(),    v_noise));
  graph->add(gtsam::PriorFactor<gtsam::Bias>(       B(ct_state), prior_state.b(),    b_noise));
  RCLCPP_INFO(logger, "[initialize]: Added nodes to 'graph'!");

  // Add initial state to the graph
  values_new.insert(    X(ct_state), prior_state.pose());
  values_new.insert(    V(ct_state), prior_state.v());
  values_new.insert(    B(ct_state), prior_state.b());
  RCLCPP_INFO(logger, "[initialize]: Inserted state to 'values_new'!");
  values_initial.insert(X(ct_state), prior_state.pose());
  values_initial.insert(V(ct_state), prior_state.v());
  values_initial.insert(B(ct_state), prior_state.b());
  RCLCPP_INFO(logger, "[initialize]: Inserted state to 'values_init'!");

  // Add ct state to map
  ct_state_lookup[timestamp] = ct_state;
  timestamp_lookup[ct_state] = timestamp;
  RCLCPP_INFO(logger, "[initialize]: Added ct_state_lookup!");

  // Clear all old imu messages (keep the last two)
  imu_times.erase(imu_times.begin(), imu_times.end() - 1);
  imu_linaccs.erase(imu_linaccs.begin(), imu_linaccs.end() - 1);
  imu_angvel.erase(imu_angvel.begin(), imu_angvel.end() - 1);
  imu_orientation.erase(imu_orientation.begin(), imu_orientation.end() - 1);
  RCLCPP_INFO(logger, "[initialize]: Cleared old imu messages!");

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
