#include "joint_reference_processor.h"

namespace robot_server_lib {

JointReferenceProcessor::JointReferenceProcessor(const RobotProperty& properties) {
    butterworth_filters_.reserve(ndof_robot);
    input_shapers_.reserve(ndof_robot);
    for (int i = 0; i < ndof_robot; ++i) {
        butterworth_filters_.emplace_back(properties.butterworth_filter_frequency[i]);
        input_shapers_.emplace_back(EIInputShaper::EI, properties.input_shaper_frequency[i], torque_ctrl_dt, 10 * M_PI);
    }
}

JointCommand JointReferenceProcessor::Process(const JointCommand& command, const RobotProperty& properties) {
    JointCommand reference = command;
    reference.position = interpolator_.Interpolate(command.position);
    position_before_filter_ = reference.position;
    for (int i = 0; i < ndof_robot; ++i) {
        if (properties.enable_butterworth_filter[i]) {
            reference.position[i] = butterworth_filters_[i].Apply(reference.position[i]);
        } else {
            butterworth_filters_[i].Reset();
        }
        position_after_filter_[i] = reference.position[i];
        if (properties.enable_input_shaper[i]) {
            reference.position[i] = input_shapers_[i].Apply(reference.position[i]);
        } else {
            input_shapers_[i].Reset();
        }
    }

    // Differentiate the final position so EI's position ramp is included in both derivatives.
    if (initialized_) {
        reference.velocity = (reference.position - previous_position_) / torque_ctrl_dt;
        reference.acceleration = (reference.velocity - previous_velocity_) / torque_ctrl_dt;
    } else {
        reference.velocity.setZero();
        reference.acceleration.setZero();
        initialized_ = true;
    }
    previous_position_ = reference.position;
    previous_velocity_ = reference.velocity;
    return reference;
}

void JointReferenceProcessor::Reset() {
    interpolator_.Reset();
    for (int i = 0; i < ndof_robot; ++i) {
        butterworth_filters_[i].Reset();
        input_shapers_[i].Reset();
    }
    position_before_filter_.setZero();
    position_after_filter_.setZero();
    previous_position_.setZero();
    previous_velocity_.setZero();
    initialized_ = false;
}

void JointReferenceProcessor::ResetInterpolation() { interpolator_.Reset(); }

}  // namespace robot_server_lib
