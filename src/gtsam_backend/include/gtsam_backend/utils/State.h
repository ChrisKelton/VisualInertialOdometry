#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>  // Needed for deserialization
#include <gtsam/base/MatrixSerialization.h>  // Eigen Vector3 support

namespace gtsam {

typedef imuBias::ConstantBias Bias;

class State {
  private:
    Pose3 pose_;       // Rotation from global to IMU, Position of IMU in global
    Vector3 velocity_; // Velocity of IMU in global
    Bias bias_;        // Bias of IMU

    friend class boost::serialization::access;
  public:
    // Default Constructor
    State() : pose_(Pose3()), velocity_(Vector3()), bias_(Bias()) {}

    // Copy Constructor
    State(const State& state) {
      this->pose_     = state.pose_;
      this->velocity_ = state.velocity_;
      this->bias_     = state.bias_;
    }

    // Constructor
    State(const Pose3& pose, const Vector3& velocity, const Bias& bias) {
      this->pose_     = pose;
      this->velocity_ = velocity;
      this->bias_     = bias;
    }

    // Set pose
    void set_pose(const Pose3& pose) {
      this->pose_ = pose;
    }

    // Set velocity
    void set_velocity(const Vector3& velocity) {
      this->velocity_ = velocity;
    }

    // set_bias
    void set_bias(const Bias& bias) {
      this->bias_ = bias;
    }

    // Return pose as Pose3
    const Pose3& pose() const {
      return pose_;
    }

    // Return velocity as Vector3
    const Vector3& v() const {
      return velocity_;
    }

    // Return bias as Bias
    const Bias& b() const {
      return bias_;
    }

    // Return rotation as Quaternion
    Quaternion q() const {
      return pose().rotation().toQuaternion();
    }

    // Return translation as Vector3
    Vector3 p() const {
      return pose().translation();
    }

    // Return accelerometer bias as Vector3
    const Vector3& ba() const {
      return b().accelerometer();
    }

    // Return gyroscope bias as Vector3
    const Vector3& bg() const {
      return b().gyroscope();
    }

    /// How this node gets printed in the ostream
    GTSAM_EXPORT
    friend std::ostream &operator<<(std::ostream &os, const State& state) {
        os << "[STATE]: q = " << std::fixed << state.q().x() << ", " << std::fixed << state.q().y() << ", "
                              << std::fixed << state.q().z() << ", " << std::fixed << state.q().w() << " | ";
        os << "p = "  << std::fixed << state.p()(0) << ", "  << std::fixed << state.p()(1)  << ", " << std::fixed << state.p()(2)  << " | ";
        os << "v = "  << std::fixed << state.v()(0) << ", "  << std::fixed << state.v()(1)  << ", " << std::fixed << state.v()(2)  << " | ";
        os << "ba = " << std::fixed << state.ba()(0) << ", " << std::fixed << state.ba()(1) << ", " << std::fixed << state.ba()(2) << " | ";
        os << "bg = " << std::fixed << state.bg()(0) << ", " << std::fixed << state.bg()(1) << ", " << std::fixed << state.bg()(2);
        return os;
    }

    /// Print function for this node
    void print(const std::string& s = "") const {
      std::cout << s << *this << std::endl;
    }

    template<class Archive>
    void serialize(Archive& ar, const unsigned int version) {
      ar& BOOST_SERIALIZATION_NVP(pose_);
      ar& BOOST_SERIALIZATION_NVP(velocity_);
      ar& BOOST_SERIALIZATION_NVP(bias_);
    };

    // Overload compound subtraction assignment as a member function
    State& operator-=(const State& other) {
      this->set_pose(this->pose().between(other.pose()));
      this->set_velocity(this->v() - other.v());
      this->set_bias(this->b() - other.b());
      return *this;  // Return a reference to the modified object
    }

    // Overload binary subtraction as a non-member friend function
    friend State operator-(State lhs, const State& rhs) {
      lhs -= rhs;  // Reuse the member operator-=
      return lhs;  // Return by value
    }
}; // State class

} // namespace gtsam
