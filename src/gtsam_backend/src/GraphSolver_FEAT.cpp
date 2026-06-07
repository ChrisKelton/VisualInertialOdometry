
#include <gtsam/geometry/Cal3_S2.h>

#include "GraphSolver.h"

#include <gtsam/nonlinear/Marginals.h>

// Maximum number of observations per SmartFactor. Keeping this bounded prevents
// very large cliques in the Bayes tree that become numerically ill-conditioned.
static constexpr size_t kMaxSmartObs = 30;

void GraphSolver::process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv, const rclcpp::Logger& logger) {

    RCLCPP_INFO(logger, "Adding '%zu' features for frame.", leftuv.size());

    // HANDLE_INFINITY: provides bearing-only constraints when depth is unobservable
    // (small baseline, parallel rays). Unlike ZERO_ON_DEGENERACY this still
    // constrains rotation and relative translation, preventing IMU-only divergence.
    gtsam::SmartProjectionParams smart_params;
    smart_params.setDegeneracyMode(gtsam::HANDLE_INFINITY);

    int feature_existing_cnt = 0;
    // Loop through LEFT features
    for(size_t i=0; i<leftids.size(); i++) {
        int feat_id = static_cast<int>(leftids.at(i));

        // Check to see if feature is already in the graph. The feature id is determined by some outside algorithm, and
        // we take this feature id as true.
        if(measurement_smart_lookup_left.find(feat_id) != measurement_smart_lookup_left.end()) {
            feature_existing_cnt++;
            auto old_factor = measurement_smart_lookup_left[feat_id];

            // Stop growing the factor once it reaches the observation cap; the existing
            // ISAM2 entry remains valid and continues to constrain the landmark.
            if (old_factor->keys().size() >= kMaxSmartObs) {
                RCLCPP_INFO(logger, "[process_feat_smart] Factor has maximal clique size!");
                continue;
            }

            // Existing feature: build a NEW factor with all prior observations + the
            // new one, then swap it into ISAM2 via remove+re-add.
            // We must NOT mutate the shared_ptr already inside ISAM2 — if we do,
            // GTSAM's variableIndex_ still reflects the old key set and throws
            // "indices and factors ... not consistent" on the next update().
            gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_camera);
            // gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(config->fx, config->fy, config->s, config->cx, config->cy));
            // gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(1.0, 1.0, 0.0, 0.0, 0.0));
            gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());
            gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI), gtsam::Point3(config->p_IinC0));
            // sensor_P_body = Pose3(R_C0toI, p_IinC0) is already body_P_sensor:
            //   rotation = R_C0toI (camera axes → IMU frame)
            //   translation = p_IinC0 = position of camera origin in IMU frame
            // Do NOT invert — inverting flips the translation sign (+4cm → −4cm),
            // producing a ~13-sigma systematic residual on every SmartFactor.
            SmartFactor::shared_ptr new_factor(new SmartFactor(measurementNoise, K, sensor_P_body, smart_params));

            const auto& old_keys     = old_factor->keys();
            const auto& old_measured = old_factor->measured();
            for (size_t j = 0; j < old_keys.size(); j++)
                new_factor->add(old_measured[j], old_keys[j]);
            new_factor->add(gtsam::Point2(leftuv.at(i)), X(ct_state));

            // Removed: smart_factors_to_remove_ was pushed here before the deferral check,
            // which would incorrectly queue removal of a factor not yet in ISAM2 (< 3 obs).
            // The removal is now deferred to after the keys().size() < 3 guard below.
            // if (smart_factor_isam_index_left_.count(feat_id))
            //     smart_factors_to_remove_.push_back(smart_factor_isam_index_left_[feat_id]);

            measurement_smart_lookup_left[feat_id] = new_factor;

            // Defer promotion until the factor has >= 3 observations.
            // 2-view triangulation is ill-conditioned at typical camera frame rates
            // (small baseline); the 3rd view provides enough overdetermination for a
            // reliable depth estimate before the factor enters ISAM2.
            if (new_factor->keys().size() < 3)
                continue;

            if (smart_factor_isam_index_left_.count(feat_id))
                smart_factors_to_remove_.push_back(smart_factor_isam_index_left_[feat_id]);

            smart_factor_graph_new_entries_.emplace_back(feat_id, graph_new->size());
            graph_new->push_back(new_factor);
            continue;
        }

        // New feature: record first observation but do NOT add to ISAM2 yet.
        // A single-view SmartFactor is always degenerate (can't triangulate); adding
        // 100+ of them per init update causes heap corruption inside ISAM2's internal
        // concurrent_unordered_map.  The factor is promoted to ISAM2 on its 3rd view.
        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_camera);
        // gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(config->fx, config->fy, config->s, config->cx, config->cy));

        // Measurements arrive as undistorted normalized image coordinates (b.x/b.z, b.y/b.z from liftProjective),
        // NOT pixels. Use identity calibration so GTSAM treats them as is; supplying the pixel space K would mix units
        // and triangulate to wrong depths.
        gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());
        // gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2(1.0, 1.0, 0.0, 0.0, 0.0));

        // Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
        gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI), gtsam::Point3(config->p_IinC0));
        SmartFactor::shared_ptr smartfactor_left(new SmartFactor(measurementNoise, K, sensor_P_body.inverse(), smart_params));
        measurement_smart_lookup_left[feat_id] = smartfactor_left;

        // Insert measurements to a smart factor
        smartfactor_left->add(gtsam::Point2(leftuv.at(i)), X(ct_state));
    }

    RCLCPP_INFO(logger, "[process_feat_smart] Got '%i' / '%zu' overlapping features.", feature_existing_cnt, leftids.size());
}

// void get_covariance_of_landmark(size_t landmark_id = ct_features) {
//     gtsam::NonlinearFactorGraph optimizedGraph = isam2->getFactorsUnsafe();
//
//     // Compute marginals using the current optimized graph and estimate
//     gtsam::Marginals marginals(optimizedGraph, values_initial);
//
//     // Query the covariance of a specific landmark
//     gtsam::Key
// }
