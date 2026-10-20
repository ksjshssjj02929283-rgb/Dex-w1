#pragma once
// 底盘命令通路：目标结构、来源仲裁、指令生成。
//
// 设计要点（对应《底盘控制切分到 chassis_node》§4 的 D1–D5）：
//   · 跨线程边界传的是**目标 + 配置快照**，不是最终电机指令。配置随目标一起穿过边界，
//     所以参数热更新在实时侧零同步开销，也不会出现"参数改了一半"的撕裂读。
//   · 指令生成是**纯函数**：只依赖 (目标, 反馈)，无成员状态、无分配、无锁、无日志，
//     可以直接在 1 kHz SCHED_FIFO 线程里调用（D5）。
//   · 仲裁状态只在写入侧（ROS 回调，由互斥回调组串行化）维护；实时侧管理启动/异常保持与目标时效。
//
// 不依赖 ROS，可脱离节点单测。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

#include "chassis_kinematics.h"
#include "robot_server_lib/chassis_type.h"
#include "shared_memory_def.h"

namespace chassis_node {

/// 底盘形态直接复用配置层的定义（`robot_server_lib/chassis_type.h`，零依赖头），
/// 全工程只有这一个枚举——配置读出来是它、命令配置里是它、过共享内存时 static_cast 成 int。
using ChassisType = robot_server_lib::utils::ChassisType;
using robot_server_lib::utils::ChassisTypeName;

/// 控制权来源。ParkingAction 优先于 VelocityCommand：驻车/对轮期间屏蔽速度指令。
enum class ChassisCommandSource : uint8_t { None = 0, VelocityCommand = 1, ParkingAction = 2 };

inline const char* ChassisCommandSourceName(ChassisCommandSource s) {
    switch (s) {
        case ChassisCommandSource::None:
            return "none";
        case ChassisCommandSource::VelocityCommand:
            return "velocity";
        case ChassisCommandSource::ParkingAction:
            return "parking";
    }
    return "unknown";
}

/// 全向指令生成参数
struct OmniCommandParams {
    double steer_limit_rad = 0.8 * M_PI;  // 舵角软限位
    double speed_ramp = 0.8;              // 速度趋近系数：cmd = fb + ramp·(target − fb)
    double zero_linear_mps = 1.0e-4;      // 整车零命令判据
    double zero_angular_rps = 1.0e-4;

    // —— 对齐限速（逐轮连续，替代原来"四轮都进 1.0 rad 门才给速度"的全或无门控）——
    // 舵角还没转到位时给全速，轮子速度矢量与地面实际运动方向成夹角 ⇒ 横向分量全是侧滑（摩地）。
    // 原门限 1.0 rad ≈ 57°，此时横向分量已达轮速的 sin57° ≈ 0.84 倍；而且一轮没到位就四轮全停，
    // 表现为走走停停。改成逐轮按对齐误差连续限速：
    //   误差 ≤ full  → 系数 1；full→crawl 之间线性降到 min_scale；> crawl → min_scale（爬行）
    double align_full_rad = 0.12;   // ≈7°，此内不限速
    double align_crawl_rad = 0.90;  // ≈52°，此外只剩爬行能力
    double align_min_scale = 0.08;

    // —— 轮速上限（同比缩放）——
    // 任一轮超限时**四轮同比缩放**：由逆解的线性性，这等价于同比缩放整个 twist，
    // ω/|v| 与瞬心位置完全保持。逐轮独立截断则会把转动分量削掉（实测塌 20–50%），
    // 表现为转弯半径变大、轨迹持续外偏，且没有明显症状。≤0 表示不限制。
    double max_wheel_speed_mps = 1.5;

    // —— 单轮矢量死区 ——
    // 轮子接近瞬心时 |v_i| → 0，atan2 的方向由噪声决定，舵角会来回摆。
    // 低于此速度就保持该轮当前舵角。
    double wheel_vector_deadband_mps = 0.03;
};

/// 控制权超时
struct ChassisArbiterParams {
    double velocity_timeout_sec = 0.5;
    double parking_timeout_sec = 0.30;
};

/// 随目标一起穿过线程边界的配置快照
struct ChassisCommandConfig {
    ChassisGeometry geometry{};
    ChassisCalibration calibration{};
    OmniCommandParams omni{};
    ChassisArbiterParams arbiter{};
    ChassisType chassis_type = ChassisType::Omni;
};

/// 命令目标：写入侧产生，实时侧消费
struct ChassisTarget {
    uint64_t seq = 0;
    ChassisCommandSource source = ChassisCommandSource::None;
    int64_t stamp_ns = 0;  // steady_clock，用于超时判定（不跨时钟源）

    Twist2D twist{};  // 速度级目标（机体系）

    // 电机级目标（驻车/对轮）。motor_level 为 true 时直接透传，不走逆解。
    bool motor_level = false;
    std::array<double, ChassisKinematics::kOmniWheels> motor_angle_rad{};
    std::array<double, ChassisKinematics::kOmniWheels> motor_speed_rpm{};
    int estop = 0;
    int neutral = 0;

    ChassisCommandConfig config{};
};

/// 仅由实时消费线程使用：异常恢复后，使恢复时刻及之前的目标失效。
/// 不 Reset 共享目标缓冲，避免与生产者并发写入冲突。所有时间均为 steady_clock。
class ChassisTargetRecoveryGate {
public:
    bool AllowsMotion(const ChassisTarget& target, bool motion_allowed, int64_t now_ns) {
        if (!motion_allowed) {
            was_blocked_ = true;
            return false;
        }
        if (was_blocked_) {
            accept_after_ns_ = now_ns;
            was_blocked_ = false;
        }
        return target.stamp_ns > accept_after_ns_;
    }

private:
    bool was_blocked_ = false;
    int64_t accept_after_ns_ = -1;
};

/// 实时侧读到的轮反馈（直接来自共享内存，1 kHz，比走话题新 ~10 ms）
struct WheelFeedback {
    bool valid = false;
    std::array<double, ChassisKinematics::kOmniWheels> angle_rad{};
    std::array<double, ChassisKinematics::kOmniWheels> speed_rpm{};
    double left_rpm = 0.0;
    double right_rpm = 0.0;
};

// 使用底盘自身反馈有效性，不依赖上半身 ready；四轮全部通过才可用于位置指令。
inline bool IsSteeringFeedbackValid(const WheelFeedback& fb, bool chassis_feedback_valid, double steer_limit_rad) {
    if (!chassis_feedback_valid || !std::isfinite(steer_limit_rad) || steer_limit_rad <= 0.0) return false;
    // 反馈允许超出目标软限位 2%，容纳轻微超调及数值误差；不放宽目标舵角限制。
    const double feedback_limit_rad = steer_limit_rad * 1.02;
    for (double angle : fb.angle_rad) {
        if (!std::isfinite(angle) || std::abs(angle) > feedback_limit_rad) return false;
    }
    return true;
}

// ============================================================ 角度工具

inline double WrapPi(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}
inline double AngleDiff(double a, double b) { return WrapPi(a - b); }

// ============================================================ 仲裁（写入侧）

/// 只在 ROS 回调线程使用，由互斥回调组保证串行。实时侧不碰它。
class ChassisCommandArbiter {
public:
    void SetParams(const ChassisArbiterParams& p) { params_ = p; }

    /// 速度指令能否取得控制权。驻车持权且未超时时返回 false（调用方应丢弃该条速度指令）。
    bool OfferVelocity(int64_t now_ns) {
        ReleaseIfExpired(now_ns);
        if (source_ == ChassisCommandSource::ParkingAction) {
            return false;
        }
        source_ = ChassisCommandSource::VelocityCommand;
        last_velocity_ns_ = now_ns;
        return true;
    }

    /// 驻车/对轮取得控制权（优先级更高，直接抢占）
    void OfferParking(int64_t now_ns) {
        source_ = ChassisCommandSource::ParkingAction;
        last_parking_ns_ = now_ns;
    }

    ChassisCommandSource source() const { return source_; }

private:
    void ReleaseIfExpired(int64_t now_ns) {
        const double age = static_cast<double>(now_ns - CurrentStampNs()) * 1e-9;
        if (source_ == ChassisCommandSource::ParkingAction && age >= params_.parking_timeout_sec) {
            source_ = ChassisCommandSource::None;
        } else if (source_ == ChassisCommandSource::VelocityCommand && age >= params_.velocity_timeout_sec) {
            source_ = ChassisCommandSource::None;
        }
    }
    int64_t CurrentStampNs() const {
        return source_ == ChassisCommandSource::ParkingAction ? last_parking_ns_ : last_velocity_ns_;
    }

    ChassisArbiterParams params_{};
    ChassisCommandSource source_ = ChassisCommandSource::None;
    int64_t last_velocity_ns_ = 0;
    int64_t last_parking_ns_ = 0;
};

// ============================================================ 指令生成（实时侧，纯函数）

/// 对齐限速系数：误差 ≤ full 给 1，full→crawl 线性降到 min_scale，> crawl 保持 min_scale。
/// 物理含义是"只用轮速里不打滑的那部分"，比全或无的门控平滑，也不会一轮拖累四轮。
inline double AlignSpeedScale(double abs_error_rad, const OmniCommandParams& p) {
    if (abs_error_rad <= p.align_full_rad) return 1.0;
    if (abs_error_rad >= p.align_crawl_rad) return p.align_min_scale;
    const double span = p.align_crawl_rad - p.align_full_rad;
    if (!(span > 0.0)) return p.align_min_scale;
    const double ratio = (abs_error_rad - p.align_full_rad) / span;
    return 1.0 + ratio * (p.align_min_scale - 1.0);
}

/// 目标是否已超时。超时即 idle：驱动轮速度清零，但**保留最后的舵角目标**（不以反馈覆盖驻车角度）。
inline bool IsTargetExpired(const ChassisTarget& t, int64_t now_ns) {
    if (t.source == ChassisCommandSource::None) return true;
    const double age = static_cast<double>(now_ns - t.stamp_ns) * 1e-9;
    const double limit = (t.source == ChassisCommandSource::ParkingAction) ? t.config.arbiter.parking_timeout_sec
                                                                           : t.config.arbiter.velocity_timeout_sec;
    return age >= limit;
}

struct ChassisControlDecision {
    bool feedback_valid = false;
    bool idle = true;
    bool hold_steering = true;
    int neutral = 1;
    std::array<double, OMNI_MOTOR_NUM> hold_angle_rad{};
};

/// 仅由实时线程持有，不改变共享内存协议。
/// 启动阶段禁止使能；放行后异常时保持使能，以零速和锁存的反馈舵角停车。
class ChassisControlGuard {
public:
    // robot_ready 由调用方合并整机的关节反馈/ready；纯底盘不依赖上半身状态。
    ChassisControlDecision Update(const ChassisTarget& target, bool has_target, const WheelFeedback& fb,
                                  bool chassis_feedback_valid, bool robot_ready, int64_t now_ns) {
        const bool is_omni = target.config.chassis_type == ChassisType::Omni;
        const bool steering_feedback_valid =
            IsSteeringFeedbackValid(fb, chassis_feedback_valid, target.config.omni.steer_limit_rad);
        if (is_omni && steering_feedback_valid) last_feedback_angle_ = fb.angle_rad;

        ChassisControlDecision decision;
        decision.feedback_valid = chassis_feedback_valid && (!is_omni || steering_feedback_valid);
        const bool motion_allowed = decision.feedback_valid && robot_ready;
        const int requested_neutral = has_target ? target.neutral : 0;

        if (!startup_released_) {
            // 下使能等待期间位置可能变化，首次使能必须使用最新可信反馈，而非首次收到的角度。
            if (is_omni && steering_feedback_valid) hold_angle_ = last_feedback_angle_;
            // 启动也视作恢复点：首次使能帧只保持位置，放行前收到的目标全部失效。
            recovery_gate_.AllowsMotion(target, false, now_ns);
            if (motion_allowed && requested_neutral == 0) startup_released_ = true;
        } else if (!motion_allowed && was_motion_allowed_) {
            // 异常入口只捕获一次；本周期反馈无效时 last_feedback_angle_ 仍是最后可信值。
            hold_angle_ = last_feedback_angle_;
        }

        if (!motion_allowed) hold_steering_ = true;
        const bool target_after_recovery =
            recovery_gate_.AllowsMotion(target, startup_released_ && motion_allowed, now_ns);
        decision.idle =
            !has_target || !target_after_recovery || requested_neutral != 0 || IsTargetExpired(target, now_ns);
        if (!decision.idle) hold_steering_ = false;

        decision.hold_steering = hold_steering_;
        decision.hold_angle_rad = hold_angle_;
        // 等待新目标不影响闭环使能；显式下使能仍优先。
        decision.neutral = startup_released_ ? requested_neutral : 1;
        was_motion_allowed_ = motion_allowed;
        return decision;
    }

private:
    bool startup_released_ = false;
    bool was_motion_allowed_ = false;
    bool hold_steering_ = true;
    std::array<double, OMNI_MOTOR_NUM> last_feedback_angle_{};
    std::array<double, OMNI_MOTOR_NUM> hold_angle_{};
    ChassisTargetRecoveryGate recovery_gate_;
};

/// 写共享内存前统一应用，确保速度整形或普通指令生成不能覆盖停车结果。
inline void ApplyChassisControlDecision(const ChassisControlDecision& decision, ChassisControl* out) {
    out->chassis_neutral = decision.neutral;
    if (decision.idle) {
        out->chassis_left_velocity = 0.0;
        out->chassis_right_velocity = 0.0;
        for (double& speed : out->chassis_motor_velocity) speed = 0.0;
    }
    if (out->chassis_mode == static_cast<int>(ChassisType::Omni) && decision.hold_steering) {
        std::copy(decision.hold_angle_rad.begin(), decision.hold_angle_rad.end(), out->chassis_motor_direction);
    }
}

/// 全向逆解 + 双解择近 + 对齐门控 + 速度趋近。与切分前 `ChassisVelocityControlCallback` 逐行等价。
///
/// 双解择近的代价用的是**环绕角差**——舵角可行域被限位切成单区间时这个度量会低估行程
/// （+140°→−140° 环绕给 80°、真实 280°）。这是已知缺陷，与对齐门限 1.0 rad 过松一起
/// 列在优化清单里；本次只搬位置不改行为，改动请单独提交并配实机验证。
inline void GenerateOmniCommand(const ChassisTarget& t, const WheelFeedback& fb, bool expired, ChassisControl* out) {
    const ChassisKinematics kin(t.config.geometry, t.config.calibration);
    const OmniCommandParams& p = t.config.omni;
    constexpr int N = ChassisKinematics::kOmniWheels;

    // 电机级目标：只有目标未过期且反馈可信时才直接透传。
    // 失效/过期时明确停驱并保持 Action 的舵角目标，不能落入普通 Twist 逆解。
    if (t.motor_level) {
        if (!expired && fb.valid) {
            for (int i = 0; i < N; ++i) {
                out->chassis_motor_direction[i] = t.motor_angle_rad[i];
                out->chassis_motor_velocity[i] = t.motor_speed_rpm[i];
            }
        } else {
            for (int i = 0; i < N; ++i) {
                out->chassis_motor_direction[i] = t.motor_angle_rad[i];
                out->chassis_motor_velocity[i] = 0.0;
            }
        }
        out->chassis_estop = t.estop;
        out->chassis_neutral = t.neutral;
        return;
    }

    std::array<Vec2, N> wheel_vec{};
    kin.OmniInverse(t.twist, wheel_vec);

    std::array<double, N> target_angle{};
    std::array<double, N> wheel_speed{};  // 带符号，m/s
    std::array<double, N> align_scale{};  // 对齐限速系数
    for (int i = 0; i < N; ++i) {
        const double speed = std::hypot(wheel_vec[i].x, wheel_vec[i].y);

        // 单轮矢量死区：速度太小时方向由噪声决定，保持当前舵角
        if (speed < p.wheel_vector_deadband_mps) {
            target_angle[i] = fb.angle_rad[i];
            wheel_speed[i] = 0.0;
            align_scale[i] = 1.0;
            continue;
        }

        // atan2 给的是轮子**理想真实朝向**；立即换到**编码器坐标系**（减去零位偏置），
        // 此后限位 / 择近解 / 对齐限速全部沿用原逻辑——目标角与 fb.angle_rad 此时同系。
        // 反过来把反馈换到真实朝向再比较，机械限位会用错坐标系（限位约束的是舵机行程）。
        const double a = WrapPi(kin.OmniSteerCmdFromTrue(i, std::atan2(wheel_vec[i].y, wheel_vec[i].x)));
        const double b = WrapPi(a + M_PI);
        // 两个等价表示：(s, a) 与 (−s, a±π)。先排除超出机械限位的候选，再取行程更短的。
        //
        // ⚠️ 行程必须用**不环绕**的角差：限位把可行域切成单区间 [−lim,
        // +lim]，其中两点间
        //    只能直着转，不能绕过 ±180°。用环绕角差会把 +140°→−140° 的 280° 行程算成 80°，
        //    既可能选错解，也会让"到位"判据提前成立而放行速度（摩地的来源之一）。
        const bool valid_a = std::abs(a) <= p.steer_limit_rad;
        const bool valid_b = std::abs(b) <= p.steer_limit_rad;
        const double cost_a = valid_a ? std::abs(a - fb.angle_rad[i]) : std::numeric_limits<double>::infinity();
        const double cost_b = valid_b ? std::abs(b - fb.angle_rad[i]) : std::numeric_limits<double>::infinity();
        const bool choose_a = cost_a <= cost_b;
        target_angle[i] = choose_a ? a : b;
        wheel_speed[i] = choose_a ? speed : -speed;
        align_scale[i] = AlignSpeedScale(std::abs(target_angle[i] - fb.angle_rad[i]), p);
    }

    // 轮速上限：任一轮超限时四轮**同比**缩放，保持 ω/|v| 与瞬心不变
    if (p.max_wheel_speed_mps > 0.0) {
        double peak = 0.0;
        for (int i = 0; i < N; ++i) peak = std::max(peak, std::abs(wheel_speed[i]));
        if (peak > p.max_wheel_speed_mps) {
            const double k = p.max_wheel_speed_mps / peak;
            for (int i = 0; i < N; ++i) wheel_speed[i] *= k;
        }
    }

    const bool zero_cmd =
        std::hypot(t.twist.vx, t.twist.vy) <= p.zero_linear_mps && std::abs(t.twist.wz) <= p.zero_angular_rps;
    for (int i = 0; i < N; ++i) {
        if (expired || zero_cmd) {
            // idle / 零命令：保持当前舵角，驱动轮清零
            out->chassis_motor_direction[i] = fb.angle_rad[i];
            out->chassis_motor_velocity[i] = 0.0;
        } else if (!fb.valid) {
            // 无可信反馈：先转向，不给速度
            out->chassis_motor_direction[i] = target_angle[i];
            out->chassis_motor_velocity[i] = 0.0;
        } else {
            out->chassis_motor_direction[i] = target_angle[i];
            // 对齐限速逐轮生效：没转到位的轮子自己降速，不再一轮拖累四轮
            const double target_rpm = kin.OmniSpeedToRpm(wheel_speed[i]) * align_scale[i];
            out->chassis_motor_velocity[i] = fb.speed_rpm[i] + (target_rpm - fb.speed_rpm[i]) * p.speed_ramp;
        }
    }
    out->chassis_estop = t.estop;
    out->chassis_neutral = t.neutral;
}

/// 差速逆解。idle 时左右轮清零。
inline void GenerateDiffCommand(const ChassisTarget& t, const WheelFeedback& fb, bool expired, ChassisControl* out) {
    if (expired || !fb.valid) {
        out->chassis_left_velocity = 0.0;
        out->chassis_right_velocity = 0.0;
        return;
    }
    const ChassisKinematics kin(t.config.geometry, t.config.calibration);
    kin.DiffInverse(t.twist, &out->chassis_left_velocity, &out->chassis_right_velocity);
}

/// 实时侧入口：根据底盘类型分发。**不分配、不加锁、不打日志**（D5）。
inline void GenerateChassisCommand(const ChassisTarget& t, const WheelFeedback& fb, bool expired, ChassisControl* out) {
    // 枚举值即线上取值，故直接转换；共享内存侧的 int 是与 EtherCAT 共享的 ABI
    out->chassis_mode = static_cast<int>(t.config.chassis_type);
    if (t.config.chassis_type == ChassisType::DiffDrive) {
        GenerateDiffCommand(t, fb, expired, out);
    } else {
        GenerateOmniCommand(t, fb, expired, out);
    }
}

}  // namespace chassis_node
