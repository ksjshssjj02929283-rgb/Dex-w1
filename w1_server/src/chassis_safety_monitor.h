#pragma once
// 底盘安全速度监测：实际超速、反馈失效、驱动器报故障、上半身异常 → 锁存安全状态，人工复位后才恢复，
// 复位本身不引发运动。上下半身是一体的：上半身异常时底盘一起停，清错（ClearError）时一起清。
//
// 监测器只看「车体线速度 + 车体角速度 + 各驱动轮线速度 + 驱动器是否报故障」，与底盘类型无关；
// 三项速度各自按上限的 110% 判超速、分别上报；
// 差速 / 全向的区别只在反馈到输入的换算（Make*SafetyInput）。目前只接差速，
// 全向接入时补一个换算函数即可复用同一套判定、锁存与复位。
//
// 只由 1 kHz 实时线程持有；无分配、无锁、无日志。不依赖 ROS，可脱离节点单测。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "chassis_kinematics.h"
#include "chassis_safety_limits.h"
#include "shared_memory_def.h"

namespace chassis_node {

/// 实测超过上限 10% 才判超速，给驱动器的跟踪超调留余量
constexpr double kChassisOverspeedMargin = 1.10;
/// 超速持续这么久才确认（约 10 个 10 ms 反馈周期），滤掉加减速瞬态与单帧毛刺
constexpr int64_t kChassisOverspeedConfirmNs = 100'000'000;
/// 驱动器故障码持续这么久才确认：驱动器自己报的故障没有瞬态问题，只滤单帧毛刺
constexpr int64_t kChassisDriverFaultConfirmNs = 20'000'000;
/// 底盘反馈连续无效这么久判定失效。更短的中断由共享内存新鲜度门处理（零速、不锁存）
constexpr int64_t kChassisFeedbackLostNs = 100'000'000;
/// 复位后判"零速指令"的阈值 (m/s, rad/s)
constexpr double kChassisZeroCommandEps = 1.0e-3;
/// 清错后等异常消失的时间：驱动器复位后约 1 s 才重新使能，上半身清错也要时间，按一次清错就能一起解除
constexpr int64_t kChassisResetWindowNs = 2'000'000'000;

/// 清错窗口的结果：解除 / 到期仍有异常被拒
enum class ChassisResetResult : uint8_t { None, Accepted, Rejected };

enum class ChassisSafetyFault : uint8_t {
    None = 0,
    LinearOverspeed = 1,
    AngularOverspeed = 2,
    WheelOverspeed = 3,
    FeedbackLost = 4,
    DriverFault = 5,
    UpperBodyFault = 6,
};

inline const char* ChassisSafetyFaultName(ChassisSafetyFault f) {
    switch (f) {
        case ChassisSafetyFault::None:
            return "none";
        case ChassisSafetyFault::LinearOverspeed:
            return "linear_overspeed";
        case ChassisSafetyFault::AngularOverspeed:
            return "angular_overspeed";
        case ChassisSafetyFault::WheelOverspeed:
            return "wheel_overspeed";
        case ChassisSafetyFault::FeedbackLost:
            return "feedback_lost";
        case ChassisSafetyFault::DriverFault:
            return "driver_fault";
        case ChassisSafetyFault::UpperBodyFault:
            return "upper_body_fault";
    }
    return "unknown";
}

/// 监测输入：本周期的底盘反馈，已换算成与底盘类型无关的量
struct ChassisSafetyInput {
    bool feedback_valid = false;                           // 共享内存帧新鲜且底盘时间戳有效
    std::array<double, OMNI_MOTOR_NUM> wheel_speed_mps{};  // 各驱动轮线速度，用不到的轮保持 0
    double linear_speed_mps = 0.0;                         // 车体线速度大小
    double wz_rps = 0.0;                                   // 车体角速度
    bool driver_fault = false;                             // 驱动器报故障
    bool upper_body_fault = false;                         // 上半身异常（关节故障、掉线），与底盘反馈无关
};

/// 差速：左右轮 rpm 换算成轮速、车体线速度与角速度；任一侧错误码非零即驱动器故障
inline ChassisSafetyInput MakeDiffSafetyInput(bool feedback_valid,
                                              const ChassisFeedback& fb,
                                              const ChassisKinematics& kin) {
    ChassisSafetyInput in;
    in.feedback_valid = feedback_valid;
    const double v_l = kin.DiffRpmToSpeed(fb.left_velocity);
    const double v_r = kin.DiffRpmToSpeed(fb.right_velocity);
    in.wheel_speed_mps[0] = v_l;
    in.wheel_speed_mps[1] = v_r;
    const Twist2D body = kin.DiffForward(v_l, v_r);
    in.linear_speed_mps = std::abs(body.vx);
    in.wz_rps = body.wz;
    in.driver_fault = fb.error_code_left != 0 || fb.error_code_right != 0;
    return in;
}

class ChassisSafetyMonitor {
public:
    /// 每个实时周期调用一次，返回是否处于安全状态。
    bool Update(const ChassisSafetyInput& in, int64_t now_ns) {
        bool feedback_ok = in.feedback_valid && std::isfinite(in.linear_speed_mps) && std::isfinite(in.wz_rps);
        double peak_wheel = 0.0;
        for (const double v : in.wheel_speed_mps) {
            feedback_ok = feedback_ok && std::isfinite(v);
            peak_wheel = std::max(peak_wheel, std::abs(v));
        }
        bool linear_over = false;
        bool angular_over = false;
        bool wheel_over = false;
        bool driver_fault = false;
        if (feedback_ok) {
            armed_ = true;
            linear_over = std::abs(in.linear_speed_mps) > kChassisSafeLinearSpeedMps * kChassisOverspeedMargin;
            angular_over = std::abs(in.wz_rps) > kChassisSafeAngularRps * kChassisOverspeedMargin;
            wheel_over = peak_wheel > kChassisSafeWheelSpeedMps * kChassisOverspeedMargin;
            driver_fault = in.driver_fault;
        }
        // 上电后还没收到过有效反馈时不判失效：此时底盘本就因反馈无效而零速
        const bool feedback_lost = armed_ && !feedback_ok;

        Confirm(linear_over, kChassisOverspeedConfirmNs, ChassisSafetyFault::LinearOverspeed, &linear_over_since_ns_,
                now_ns);
        Confirm(angular_over, kChassisOverspeedConfirmNs, ChassisSafetyFault::AngularOverspeed, &angular_over_since_ns_,
                now_ns);
        Confirm(wheel_over, kChassisOverspeedConfirmNs, ChassisSafetyFault::WheelOverspeed, &wheel_over_since_ns_,
                now_ns);
        Confirm(driver_fault, kChassisDriverFaultConfirmNs, ChassisSafetyFault::DriverFault, &driver_fault_since_ns_,
                now_ns);
        Confirm(feedback_lost, kChassisFeedbackLostNs, ChassisSafetyFault::FeedbackLost, &feedback_lost_since_ns_,
                now_ns);
        Confirm(in.upper_body_fault, kChassisDriverFaultConfirmNs, ChassisSafetyFault::UpperBodyFault,
                &upper_body_fault_since_ns_, now_ns);
        fault_present_ = linear_over || angular_over || wheel_over || driver_fault || feedback_lost || in.upper_body_fault;
        present_fault_ = linear_over      ? ChassisSafetyFault::LinearOverspeed
                         : angular_over   ? ChassisSafetyFault::AngularOverspeed
                         : wheel_over     ? ChassisSafetyFault::WheelOverspeed
                         : driver_fault   ? ChassisSafetyFault::DriverFault
                         : feedback_lost  ? ChassisSafetyFault::FeedbackLost
                         : in.upper_body_fault ? ChassisSafetyFault::UpperBodyFault
                                               : ChassisSafetyFault::None;
        // 清错窗口内每周期重试：异常一消失就解除；到期仍有异常则放弃，保持锁存
        if (reset_pending_) {
            if (TryReset()) {
                reset_pending_ = false;
                reset_result_ = ChassisResetResult::Accepted;
            } else if (!latched_ || now_ns >= reset_deadline_ns_) {
                reset_pending_ = false;
                if (latched_) reset_result_ = ChassisResetResult::Rejected;
            }
        }
        return latched_;
    }

    /// 人工复位：处于安全状态、且当前没有任何异常条件时才解除，返回是否解除。
    bool TryReset() {
        if (!latched_ || fault_present_) return false;
        latched_ = false;
        fault_ = ChassisSafetyFault::None;
        awaiting_zero_command_ = true;
        return true;
    }

    /// 复位后的放行门：复位本身不能引发运动。解除锁存后先保持零速，收到零速指令、
    /// 或上层停发指令（目标超时）后才放行；之前收到的非零速度不执行，返回 true 表示仍需保持。
    /// 遥控需要先回零，任务层需要先停止任务。
    bool HoldAfterReset(bool command_active, double vx, double wz) {
        if (!awaiting_zero_command_) return false;
        const bool zero_command = std::abs(vx) <= kChassisZeroCommandEps && std::abs(wz) <= kChassisZeroCommandEps;
        if (!command_active || zero_command) {
            awaiting_zero_command_ = false;
            return false;
        }
        return true;
    }
    bool awaiting_zero_command() const { return awaiting_zero_command_; }

    bool latched() const { return latched_; }
    /// 首个触发安全状态的异常；同一周期多项同时确认时，按线速度、角速度、单轮、驱动器、反馈、上半身的顺序取第一个
    ChassisSafetyFault fault() const { return fault_; }
    /// 本周期是否仍存在异常条件（不论是否已确认）
    bool fault_present() const { return fault_present_; }

    /// 清错请求（ClearError 上升沿）：处于安全状态时开一个窗口，之后每周期重试复位，异常消失即解除。
    void RequestReset(int64_t now_ns) {
        if (!latched_) return;
        reset_pending_ = true;
        reset_deadline_ns_ = now_ns + kChassisResetWindowNs;
    }
    bool reset_pending() const { return reset_pending_; }
    /// 清错结果，取一次即清空，供调用方打一次日志
    ChassisResetResult TakeResetResult() {
        const ChassisResetResult r = reset_result_;
        reset_result_ = ChassisResetResult::None;
        return r;
    }
    /// 本周期仍存在的异常（按线速度、角速度、单轮、驱动器、反馈、上半身的顺序取第一个），用于说明拒绝原因
    ChassisSafetyFault present_fault() const { return present_fault_; }

private:
    void Confirm(bool condition, int64_t confirm_ns, ChassisSafetyFault fault, int64_t* since_ns, int64_t now_ns) {
        if (!condition) {
            *since_ns = -1;
            return;
        }
        if (*since_ns < 0) *since_ns = now_ns;
        if (!latched_ && now_ns - *since_ns >= confirm_ns) {
            latched_ = true;
            fault_ = fault;
        }
    }

    bool armed_ = false;
    bool latched_ = false;
    bool awaiting_zero_command_ = false;
    bool fault_present_ = false;
    bool reset_pending_ = false;
    int64_t reset_deadline_ns_ = 0;
    ChassisResetResult reset_result_ = ChassisResetResult::None;
    ChassisSafetyFault present_fault_ = ChassisSafetyFault::None;
    ChassisSafetyFault fault_ = ChassisSafetyFault::None;
    int64_t linear_over_since_ns_ = -1;
    int64_t angular_over_since_ns_ = -1;
    int64_t wheel_over_since_ns_ = -1;
    int64_t driver_fault_since_ns_ = -1;
    int64_t feedback_lost_since_ns_ = -1;
    int64_t upper_body_fault_since_ns_ = -1;
};

}  // namespace chassis_node
