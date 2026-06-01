
#include "GraphSolver.h"

void GraphSolver::process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv) {

    for(size_t i=0; i<leftids.size(); i++) {
        int feat_id = static_cast<int>(leftids.at(i));

        if(measurement_smart_lookup_left.find(feat_id) != measurement_smart_lookup_left.end()) {
            // Existing feature: build a NEW factor with all prior observations + the
            // new one, then swap it into ISAM2 via remove+re-add.
            // We must NOT mutate the shared_ptr already inside ISAM2 — if we do,
            // GTSAM's variableIndex_ still reflects the old key set and throws
            // "indices and factors ... not consistent" on the next update().
            auto old_factor = measurement_smart_lookup_left[feat_id];

            gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_camera);
            gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());
            gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI), gtsam::Point3(config->p_IinC0));
            SmartFactor::shared_ptr new_factor(new SmartFactor(measurementNoise, K, sensor_P_body.inverse()));

            const auto& old_keys    = old_factor->keys();
            const auto& old_measured = old_factor->measured();
            for (size_t j = 0; j < old_keys.size(); j++)
                new_factor->add(old_measured[j], old_keys[j]);
            new_factor->add(gtsam::Point2(leftuv.at(i)), X(ct_state));

            if (smart_factor_isam_index_left_.count(feat_id))
                smart_factors_to_remove_.push_back(smart_factor_isam_index_left_[feat_id]);

            measurement_smart_lookup_left[feat_id] = new_factor;
            smart_factor_graph_new_entries_.emplace_back(feat_id, graph_new->size());
            graph_new->push_back(new_factor);
            continue;
        }

        // New feature: record first observation but do NOT add to ISAM2 yet.
        // A single-view SmartFactor is always degenerate (can't triangulate); adding
        // 100+ of them per init update causes heap corruption inside ISAM2's internal
        // concurrent_unordered_map.  The factor is promoted to ISAM2 on its 2nd view.
        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_camera);
        gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());
        gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI), gtsam::Point3(config->p_IinC0));
        SmartFactor::shared_ptr smartfactor_left(new SmartFactor(measurementNoise, K, sensor_P_body.inverse()));
        measurement_smart_lookup_left[feat_id] = smartfactor_left;
        smartfactor_left->add(gtsam::Point2(leftuv.at(i)), X(ct_state));
    }
}

