
#include "GraphSolver.h"

void GraphSolver::process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv) {

    for(size_t i=0; i<leftids.size(); i++) {
        int feat_id = static_cast<int>(leftids.at(i));

        if(measurement_smart_lookup_left.find(feat_id) != measurement_smart_lookup_left.end()) {
            // Insert measurements to a smart factor
            measurement_smart_lookup_left[feat_id]->add(gtsam::Point2(leftuv.at(i)), X(ct_state));
            continue;
        }

        // If we know it is not in the graph
        // Create a smart factor for the new feature
        gtsam::noiseModel::Isotropic::shared_ptr measurementNoise = gtsam::noiseModel::Isotropic::Sigma(2, config->sigma_camera);
        gtsam::Cal3_S2::shared_ptr K(new gtsam::Cal3_S2());

        // Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
        gtsam::Pose3 sensor_P_body = gtsam::Pose3(gtsam::Rot3(config->R_C0toI), gtsam::Point3(config->p_IinC0));
        SmartFactor::shared_ptr smartfactor_left(new SmartFactor(measurementNoise, K, sensor_P_body.inverse()));
        // We have a Projection Pose factor described by some camera observing a specific point. (We can add multiple
        // cameras/poses to the factor graph, but we are just adding additional landmarks). The SmartFactor allows us
        // to just store the 2D points of a landmark and use triangulation to triangulate the 3D points of each
        // point online. I think b/c of that, we will only have valid measurements when we have seen the same landmark
        // 3 times?
        measurement_smart_lookup_left[feat_id] = smartfactor_left;

        // Insert measurements to a smart factor
        smartfactor_left->add(gtsam::Point2(leftuv.at(i)), X(ct_state));

        // Add smart factor to FORSTER2 model
        graph_new->push_back(smartfactor_left);
        graph->push_back(smartfactor_left);
    }
}

