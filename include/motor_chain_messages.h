// Map motor status and command structs to eigen vectors for convenient matrix math
// e.g. motor_chain_command.motor_position() = kp*(desired - motor_chain_status.motor_position());
// plus some helpers like velocity
// See motor_messages.h for value types

#include "motor_messages/motor_messages.h"
#include <Eigen/Dense>
#include <vector>

// for eigen map to work the structs must be even multiples of float
static_assert(sizeof(MotorStatus) % sizeof(float) == 0);
static_assert(sizeof(MotorCommand) % sizeof(float) == 0);

struct MotorChainStatus{
 private:
    typedef Eigen::Map<const Eigen::VectorXf, Eigen::Unaligned, 
        Eigen::InnerStride<sizeof(MotorStatus)/sizeof(float)>> ConstMotorStatusFloat;
    typedef Eigen::Map<const Eigen::Matrix<uint32_t, Eigen::Dynamic, 1>, Eigen::Unaligned, 
        Eigen::InnerStride<sizeof(MotorStatus)/sizeof(float)>> ConstMotorStatusTimestamp;    
 public:
    MotorChainStatus(std::vector<MotorStatus> &sv) :
        MotorChainStatus(sv.data(), sv.size()) {}
    MotorChainStatus(MotorStatus* sa, int num_motors) :
        size_(num_motors),
        data_(sa),
        mcu_timestamp_(&sa[0].mcu_timestamp, size()),
        motor_position_(&sa[0].motor_position, size()),
        joint_position_(&sa[0].joint_position, size()),
        iq_(&sa[0].iq, size()),
        torque_(&sa[0].torque, size()) {}
    int size() const { return size_; }
    ConstMotorStatusTimestamp const& mcu_timestamp() const { return mcu_timestamp_; }
    ConstMotorStatusFloat const& motor_position() const { return motor_position_; }
    ConstMotorStatusFloat const& joint_position() const { return joint_position_; }
    ConstMotorStatusFloat const& iq() const { return iq_; }
    ConstMotorStatusFloat const& torque() const { return torque_; }
    Eigen::VectorXd dt_seconds(const MotorChainStatus& last) {
        //TODO unhardcode cpu frequency
        return (mcu_timestamp() - last.mcu_timestamp()).cast<double>()/170e6; 
    }
    Eigen::VectorXd motor_velocity(const MotorChainStatus& last) {
        return (motor_position() - last.motor_position()).cast<double>().array()/dt_seconds(last).array();
    }
    Eigen::VectorXd joint_velocity(const MotorChainStatus& last) {
        return (joint_position() - last.joint_position()).cast<double>().array()/dt_seconds(last).array();
    }
 private:
    int size_;
    MotorStatus *data_;
    ConstMotorStatusTimestamp mcu_timestamp_;
    ConstMotorStatusFloat motor_position_;
    ConstMotorStatusFloat joint_position_;
    ConstMotorStatusFloat iq_;
    ConstMotorStatusFloat torque_;
};

struct MotorChainCommand {
 private:
    typedef Eigen::Map<Eigen::VectorXf, Eigen::Unaligned, 
        Eigen::InnerStride<sizeof(MotorCommand)/sizeof(float)>> MotorCommandFloat;
    typedef Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>, Eigen::Unaligned, 
        Eigen::InnerStride<sizeof(MotorCommand)/sizeof(uint8_t)>> MotorCommandMode;

 public:
    MotorChainCommand(std::vector<MotorCommand> &cv) :
        MotorChainCommand(cv.data(), cv.size()) {}
    MotorChainCommand(MotorCommand* ca, int num_motors) :
        size_(num_motors),
        data_(ca),
        mode_desired_(&ca[0].mode_desired, size()),
        current_desired_(&ca[0].current_desired, size()),
        position_desired_(&ca[0].position_desired, size()),
        velocity_desired_(&ca[0].velocity_desired, size()),
        torque_desired_(&ca[0].torque_desired, size()) {}
    int size() const { return size_; }
    MotorCommandMode& mode_desired() { return mode_desired_; }
    MotorCommandFloat& current_desired() { return current_desired_; }
    MotorCommandFloat& position_desired() { return position_desired_; }
    MotorCommandFloat& velocity_desired() { return velocity_desired_; }
    MotorCommandFloat& torque_desired() { return torque_desired_; }

 private:
    int size_;
    MotorCommand *data_;
    MotorCommandMode mode_desired_;
    MotorCommandFloat current_desired_;
    MotorCommandFloat position_desired_;
    MotorCommandFloat velocity_desired_;
    MotorCommandFloat torque_desired_;
};