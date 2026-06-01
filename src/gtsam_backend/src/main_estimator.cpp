/*
 * Lu Gan
 * ganlu@umich.edu
 */

#include <vector>
#include <signal.h>
#include <execinfo.h>
#include <unistd.h>
#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "GraphSolver.h"
#include "utils/Config.h"
#include "utils/Convert.h"

class VioNode : public rclcpp::Node {
public:
    VioNode() : Node("vio") {
        config = new Config();
        setup_config();
        setup_subpub();

        graphsolver = new GraphSolver(config);
    }

    ~VioNode() {
        delete config;
        delete graphsolver;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subIMUMeas;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr subUVMeas;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pubPoseIMU;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPathIMU;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFeatureCloud;

    Config* config;
    GraphSolver* graphsolver;
    int poses_seq = 0;
    int skip = 0;

    void setup_config() {
        config->fixedId = "global";
        // declare_parameter is specific to rclcpp::Node.
        config->fixedId = this->declare_parameter<std::string>("fixedId", config->fixedId);

        std::vector<double> gravity = {0, 0, 9.8};
        gravity = this->declare_parameter<std::vector<double>>("gravity", gravity);
        for (size_t i = 0; i < 3; ++i) config->gravity(i, 0) = gravity.at(i);

        config->imuWait = this->declare_parameter<int>("imuWait", 300);
        config->featWait = this->declare_parameter<int>("featWait", 0);

        std::vector<double> R_C0toI = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        R_C0toI = this->declare_parameter<std::vector<double>>("R_C0toI", R_C0toI);
        for (size_t i = 0; i < 9; ++i) config->R_C0toI(i) = R_C0toI.at(i);

        std::vector<double> p_IinC0 = {0, 0, 0};
        p_IinC0 = this->declare_parameter<std::vector<double>>("p_IinC0", p_IinC0);
        for (size_t i = 0; i < 3; ++i) config->p_IinC0(i) = p_IinC0.at(i);

        std::vector<double> prior_qGtoI = {0, 0, 0, 1.0};
        prior_qGtoI = this->declare_parameter<std::vector<double>>("prior_qGtoI", prior_qGtoI);
        for (size_t i = 0; i < 4; ++i) config->prior_qGtoI(i) = prior_qGtoI.at(i);

        std::vector<double> prior_pIinG = {0, 0, 0};
        prior_pIinG = this->declare_parameter<std::vector<double>>("prior_pIinG", prior_pIinG);
        for (size_t i = 0; i < 3; ++i) config->prior_pIinG(i) = prior_pIinG.at(i);

        std::vector<double> prior_vIinG = {0, 0, 0};
        prior_vIinG = this->declare_parameter<std::vector<double>>("prior_vIinG", prior_vIinG);
        for (size_t i = 0; i < 3; ++i) config->prior_vIinG(i) = prior_vIinG.at(i);

        std::vector<double> prior_ba = {0, 0, 0};
        prior_ba = this->declare_parameter<std::vector<double>>("prior_ba", prior_ba);
        for (size_t i = 0; i < 3; ++i) config->prior_ba(i) = prior_ba.at(i);

        std::vector<double> prior_bg = {0, 0, 0};
        prior_bg = this->declare_parameter<std::vector<double>>("prior_bg", prior_bg);
        for (size_t i = 0; i < 3; ++i) config->prior_bg(i) = prior_bg.at(i);

        config->sigma_camera = this->declare_parameter<double>("sigma_camera", 1.0/484.1316);
        config->sigma_camera_sq = std::pow(config->sigma_camera, 2);

        config->sigma_a = this->declare_parameter<double>("accelerometer_noise_density", 0.01);
        config->sigma_a_sq = std::pow(config->sigma_a, 2);
        config->sigma_g = this->declare_parameter<double>("gyroscope_noise_density", 0.005);
        config->sigma_g_sq = std::pow(config->sigma_g, 2);
        config->sigma_wa = this->declare_parameter<double>("accelerometer_random_walk", 0.002);
        config->sigma_wa_sq = std::pow(config->sigma_wa, 2);
        config->sigma_wg = this->declare_parameter<double>("gyroscope_random_walk", 4.0e-06);
        config->sigma_wg_sq = std::pow(config->sigma_wg, 2);

        config->sigma_prior_rotation    = this->declare_parameter<double>("sigma_prior_rotation",    1.0e-4);
        config->sigma_prior_translation = this->declare_parameter<double>("sigma_prior_translation", 1.0e-4);
        config->sigma_velocity          = this->declare_parameter<double>("sigma_velocity",          1.0e-4);
        config->sigma_bias              = this->declare_parameter<double>("sigma_bias",              1.0e-4);
        config->sigma_pose_rotation     = this->declare_parameter<double>("sigma_pose_rotation",     1.0e-4);
        config->sigma_pose_translation  = this->declare_parameter<double>("sigma_pose_translation",  1.0e-4);

        Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
        RCLCPP_INFO(this->get_logger(), "fixed ID: %s", config->fixedId.c_str());
        RCLCPP_INFO(this->get_logger(), "imuWait: %d, featWait: %d", config->imuWait, config->featWait);
    }

    void setup_subpub() {
        subIMUMeas = this->create_subscription<sensor_msgs::msg::Imu>(
            "vio/data_imu", 2000,
            std::bind(&VioNode::handle_measurement_imu, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing: vio/data_imu");

        subUVMeas = this->create_subscription<sensor_msgs::msg::PointCloud>(
            "vio/data_uv", 500,
            std::bind(&VioNode::handle_measurement_uv, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribing: vio/data_uv");

        pubPoseIMU = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("vio/pose_imu", 2);
        RCLCPP_INFO(this->get_logger(), "Publishing: vio/pose_imu");

        pubPathIMU = this->create_publisher<nav_msgs::msg::Path>("vio/path_imu", 2);
        RCLCPP_INFO(this->get_logger(), "Publishing: vio/path_imu");

        pubFeatureCloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("vio/feature_cloud", 2);
        RCLCPP_INFO(this->get_logger(), "Publishing: vio/feature_cloud");
    }

    void handle_measurement_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 500, "Ingesting new imu measurement, '%zu' total measurements", graphsolver->get_imu_times().size());
        Eigen::Vector3d linearacceleration;
        linearacceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        Eigen::Vector3d angularvelocity;
        angularvelocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        Eigen::Vector4d orientation;
        orientation << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;

        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        graphsolver->addmeasurement_imu(timestamp, linearacceleration, angularvelocity, orientation);
    }

    void handle_measurement_uv(const sensor_msgs::msg::PointCloud::ConstSharedPtr msg) {
        if (graphsolver->is_initialized() && skip != config->featWait) {
            skip++;
            return;
        } else
            skip = 0;

        std::vector<uint> leftids;
        std::vector<Eigen::Vector2d> leftuv;

        for(size_t i = 0; i < msg->points.size(); ++i) {
            int v = msg->channels[0].values[i] + 0.5;
            int id = v / 1;
            leftids.push_back((uint)id);
            Eigen::Vector2d uv;
            uv << msg->points[i].x, msg->points[i].y;
            leftuv.push_back(uv);
        }

        RCLCPP_INFO(this->get_logger(), "Adding '%zu' features for frame.", leftuv.size());
        double timestamp = rclcpp::Time(msg->header.stamp).seconds();
        graphsolver->addmeasurement_uv(timestamp, leftids, leftuv, this->get_logger());
        optimize_graph(timestamp);
    }

    void optimize_graph(double timestamp) {
        graphsolver->optimize(this->get_logger());

        gtsam::State state = graphsolver->get_current_state();
        publish_state(timestamp, state);
        std::vector<std::pair<double, gtsam::State>> trajectory = graphsolver->get_trajectory(this->get_logger());
        if (!trajectory.empty()) {
            const auto& p = trajectory.back().second.p();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[VIO] t=%.1fs  states=%zu  pos=(%.2f, %.2f, %.2f)",
                timestamp, trajectory.size(), p(0), p(1), p(2));
        }
        publish_trajectory(timestamp, trajectory);
        publish_cloud(timestamp);
    }

    void publish_state(double timestamp, gtsam::State& state) {
        if(!graphsolver->is_initialized())
            return;

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
        pose.header.frame_id = config->fixedId;
        Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double,6,6>::Zero();
        ToPoseWithCovariance(state.pose(), covariance, pose.pose);
        pubPoseIMU->publish(pose);
    }

    void publish_trajectory(double timestamp, Trajectory& trajectory) {
        if (trajectory.empty())
            return;

        std::vector<geometry_msgs::msg::PoseStamped> traj_est;
        for (auto it = trajectory.begin(); it != trajectory.end(); ++it) {
            geometry_msgs::msg::PoseStamped poseStamped;
            poseStamped.header.stamp = rclcpp::Time(static_cast<int64_t>(it->first * 1e9));
            poseStamped.header.frame_id = config->fixedId;
            ToPose(it->second.pose(), poseStamped.pose);
            traj_est.push_back(poseStamped);
        }

        nav_msgs::msg::Path patharr;
        patharr.header.frame_id = config->fixedId;
        patharr.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
        patharr.poses = traj_est;
        pubPathIMU->publish(patharr);
    }

    void publish_cloud(double timestamp) {
        if (!graphsolver->is_initialized())
            return;

        std::vector<Eigen::Vector3d> points = graphsolver->get_current_features();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (size_t i = 0; i < points.size(); ++i) {
            pcl::PointXYZ pt;
            pt.x = points.at(i)(0);
            pt.y = points.at(i)(1);
            pt.z = points.at(i)(2);
            cloud->push_back(pt);
        }

        sensor_msgs::msg::PointCloud2 msgOut;
        pcl::toROSMsg(*cloud, msgOut);
        msgOut.header.frame_id = config->fixedId;
        msgOut.header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));
        pubFeatureCloud->publish(msgOut);
    }
};

static void segfault_handler(int sig) {
    void* array[30];
    size_t size = backtrace(array, 30);
    fprintf(stderr, "\n[VIO CRASH] Caught signal %d (SIGSEGV). Stack trace:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    _exit(1);
}

int main(int argc, char** argv) {
    signal(SIGSEGV, segfault_handler);
    signal(SIGABRT, segfault_handler);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VioNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
