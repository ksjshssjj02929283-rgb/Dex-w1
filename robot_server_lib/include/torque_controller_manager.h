#pragma once

#include "pinocchio_include.h"
#include "torque_controller_include.h"
#include "utilities.h"

namespace robot_server_lib
{

inline const char *ToString(TorqueControllerState state)
{
    switch (state)
    {
    case TorqueControllerState::Init:
        return "Init";
    case TorqueControllerState::Idle:
        return "Idle";
    case TorqueControllerState::Decelerating:
        return "Decelerating";
    case TorqueControllerState::Operating:
        return "Operating";
    default:
        return "Unknown";
    }
}

struct ProcessResponse
{
    bool re_run{false};
};

using namespace robot_server_lib;

class TorqueControllerManager
{
   public:
    TorqueControllerManager(RobotModel robot_model, PlannerManagerBuffers manager_buffers, TorqueControlBuffer robot_buffers);
    ~TorqueControllerManager();

    void LoopOnce();

   private:
    void Initialize();

    ProcessResponse InitProcess();
    ProcessResponse IdleProcess();
    ProcessResponse DeceleratingProcess();
    ProcessResponse OperatingProcess();

    void ChangeState(TorqueControllerState new_state);

    RobotModel robot_model_;
    PlannerManagerBuffers manager_buffers_;
    TorqueControlBuffer robot_buffers_;

    TorqueControllerState controller_state_{TorqueControllerState::Init};

    std::shared_ptr<TorqueControllerBase> torque_controller_{nullptr};

    // Runtime data
    Int64 current_episode_index_{-1};
    Int64 last_cmd_sequence_number_{-1};
    struct
    {
        // Timing maintained by the state machine
        Int64 loop_counter{-1};
        TimeMetaInfo time_meta{};

        JointRawMeasurement raw_measurement{};
        TorqueCtrlCommand output_cmd{};
        struct
        {
            PlannerManagerCommand cmd{};
            TimePointSystem time_point{TimePointSystem::Invalid()};
        } input_joint_cmd;
    } runtime_data_;
    static constexpr int MaxDelayInNs = 40 * 1000 * 1000;
};

}  // namespace robot_server_lib
