#pragma once

#include "cartesian_impedance_controller.h"
#include "motion_planner_include.h"
#include "safety_check.h"
#include "utils.h"

namespace robot_server_lib {

struct RequestResponse {
    bool re_run{false};
    bool request_consumed{false};
};

class PlannerManager {
public:
    PlannerManager(RobotModel robot_model,
                   PlannerManagerBuffers buffer,
                   std::function<void(Int64, RequestType, RequestStatus, Status)> report_request_status,
                   bool is_sim = false);

    ~PlannerManager();

    void LoopOnce();

private:
    RobotModel robot_model_;

    std::function<void(Int64, RequestType, RequestStatus, Status)> ReportRequestStatus;

    bool is_sim_{false};

    // Property
    RobotProperty robot_property_;
    RunMode previous_run_mode_{RunMode::Position};

    // StateMachine
    Status manager_state_{Status::Init};

    // Buffers for communication with other threads
    PlannerManagerBuffers manager_buffer_;

    // Current episode index
    Int64 current_episode_{-1};

    // Planners for different tasks
    std::shared_ptr<MotionPlannerBase> current_planner_{nullptr};
    std::shared_ptr<IdlePlanner> idle_planner_{nullptr};
    std::shared_ptr<MoveJointPlanner> move_joint_planner_{nullptr};
    std::shared_ptr<MoveLinePlanner> move_line_planner_{nullptr};
    std::shared_ptr<MoveTrajPlanner> move_traj_planner_{nullptr};
    std::shared_ptr<JointStreamingPlanner> joint_streaming_planner_{nullptr};
    std::shared_ptr<EEPoseStreamingPlanner> ee_pose_streaming_planner_{nullptr};
    std::shared_ptr<EEVelocityStreamingPlanner> ee_velocity_streaming_planner_{nullptr};
    std::shared_ptr<dynamics::CartesianImpedanceController> impedance_ctrl_primary_{nullptr};
    std::shared_ptr<dynamics::CartesianImpedanceController> impedance_ctrl_secondary_{nullptr};
    struct {
        // Timing maintained by the state machine
        Int64 loop_counter{-1};
        TimeMetaInfo time_meta{};

        // buffers
        PlannerRequest planner_request{};
        PlannerRequest emergency_request{};
        JointMeasurement measurement{};
        JointCommand joint_cmd{};
        Eigen::VectorXd joint_position_primary;
        Eigen::VectorXd joint_position_secondary;
        Int64 latest_processing_request_id{-1};
        Int64 last_finished_request_id{-1};
        Int64 terminated_request_id{-1};
        RequestType latest_processing_request_type{RequestType::Empty};

        // Pre-allocated variables for PublishPlannerInfo (to reduce memory allocation)
        Eigen::VectorXd publish_measurement_primary;
        Eigen::VectorXd publish_measurement_secondary;
        Eigen::VectorXd publish_measurement_torso;
        Eigen::VectorXd publish_ee_pose;
        Eigen::VectorXd publish_torso_X_Z_RY;
    } runtime_data_;

    void Initialize();
    void ChangeState(Status new_state);
    void SetProperty(const PlannerRequest& request);

    RequestResponse OnRequest();
    // Process the request based on the current status
    RequestResponse OnRequestStateInit();
    RequestResponse OnRequestStateIdle();
    RequestResponse OnRequestStateRunning();
    RequestResponse OnRequestStateError();

    void OnEmergencyRequest();
    void ConsumeRequest(PlannerRequest* request);
    void PublishPlannerInfo();
};

}  // namespace robot_server_lib
