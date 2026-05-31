
#include "GraphSolver.h"

void GraphSolver::process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv) {

    for(size_t i=0; i<leftids.size(); i++) {
        int feat_id = static_cast<int>(leftids.at(i));

        if(measurement_smart_lookup_left.find(feat_id) != measurement_smart_lookup_left.end()) {
            // Existing feature: mutate the factor with the new observation, then
            // remove its old ISAM2 entry and re-add it so ISAM2's variableIndex_
            // reflects the expanded key set (otherwise SIGSEGV during next update).
            auto factor = measurement_smart_lookup_left[feat_id];
            factor->add(gtsam::Point2(leftuv.at(i)), X(ct_state));

            if (smart_factor_isam_index_left_.count(feat_id))
                smart_factors_to_remove_.push_back(smart_factor_isam_index_left_[feat_id]);

            smart_factor_graph_new_entries_.emplace_back(feat_id, graph_new->size());
            graph_new->push_back(factor);
            continue;
        }

        // New feature: create and register a fresh SmartFactor.
        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_camera);
        gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());
        gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI), gtsam::Point3(config->p_IinC0));
        SmartFactor::shared_ptr smartfactor_left(new SmartFactor(measurementNoise, K, sensor_P_body.inverse()));
        measurement_smart_lookup_left[feat_id] = smartfactor_left;
        smartfactor_left->add(gtsam::Point2(leftuv.at(i)), X(ct_state));

        smart_factor_graph_new_entries_.emplace_back(feat_id, graph_new->size());
        graph_new->push_back(smartfactor_left);
        graph->push_back(smartfactor_left);
    }
}

