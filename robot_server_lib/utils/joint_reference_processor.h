#pragma once

#include <vector>

#include "butterworth_filter.h"
#include "ei_input_shaper.h"
#include "utils.h"

namespace robot_server_lib {

class JointReferenceProcessor {
public:
    explicit JointReferenceProcessor(const RobotProperty& properties);

    JointCommand Process(const JointCommand& command, const RobotProperty& properties);
    void Reset();
    void ResetInterpolation();

    const VectorDof& PositionBeforeFilter() const { return position_before_filter_; }
    const VectorDof& PositionAfterFilter() const { return position_after_filter_; }

private:
    utils::LinearInterpolator interpolator_{manager_dt_micro / torque_ctrl_dt_micro};
    std::vector<ButterworthFilter> butterworth_filters_;
    std::vector<EIInputShaper> input_shapers_;
    VectorDof position_before_filter_{VectorDof::Zero()};
    VectorDof position_after_filter_{VectorDof::Zero()};
    VectorDof previous_position_{VectorDof::Zero()};
    VectorDof previous_velocity_{VectorDof::Zero()};
    bool initialized_{false};
};

}  // namespace robot_server_lib
