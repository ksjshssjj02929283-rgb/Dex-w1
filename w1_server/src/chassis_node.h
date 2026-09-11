#pragma once

#include <tf2_ros/transform_broadcaster.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include "chassis_ekf.h"
#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "chassis_interfaces/msg/chassis_motor_feedback.hpp"
#include "imu_processor.h"
#include "odom_mode.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "shared_memory_def.h"

namespace chassis_node {

/**
 * IMU 数据来源。由硬件版本决定，但**用运行时观测判定而非配置**：
 *   0.3.0 —— IMU 挂在独立串口，由 dexe_sensors/imu 驱动发 /imu/chassis
 *   0.3.1 —— IMU 走 EtherCAT，数据进共享内存，由 w1_server 转发 /imu/chassis
 * 两版话题同名，故不能靠话题区分；而「共享内存里 IMU 时间戳是否在变」是可直接
 * 观测且不会骗人的事实——配置可能填错，这个不会。
 */
enum class ImuSource {
    Unknown,  // 检测中
    Shm,      // 0.3.1：直接读共享内存，绕过话题（少一跳序列化 + 传输）
    Topic,    // 0.3.0：订阅 /imu/chassis
};

class ChassisNode : public rclcpp::Node {
public:
    ChassisNode();  ///< 自动从 share/w1_server/config/chassis_params.yaml 加载参数
    explicit ChassisNode(const rclcpp::NodeOptions& options);
    ~ChassisNode() override = default;

    // 禁止拷贝与赋值
    ChassisNode(const ChassisNode&) = delete;
    ChassisNode& operator=(const ChassisNode&) = delete;

    void ResetOdometry();

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    void ImuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    /// IMU 处理主体。两条来源路径（shm / 话题）都汇到这里，保证行为一致。
    void ProcessImu(const sensor_msgs::msg::Imu& imu);

    /**
     * 判定 IMU 来源。仅在 Unknown 状态下工作，一旦定下不再改变。
     *
     * 判据是**共享内存里的 IMU 时间戳是否在变化**，而非「是否非零」——
     * 启动瞬间 EtherCAT 可能尚未就绪，此时 shm 全零会被误判成 0.3.0。
     * 故给一个宽限期轮询：期内观察到变化即 Shm，超时未变则 Topic。
     */
    void DetectImuSource(const ImuData& imu_data, bool feedback_valid);

    /// 把共享内存的 ImuData 转成 ROS 消息。
    /// ⚠️ stamp 用节点时钟，**不用 ImuData::timestamp**——后者是 HI91 协议时间戳
    /// （传感器自身计数器，uint32 会溢出），既非 REALTIME 也非 MONOTONIC，
    /// 只能用来判断「变了没有」，不能当时间用。
    sensor_msgs::msg::Imu ConvertShmImu(const ImuData& imu_data);

    // 主计时器：按固定频率积分与发布里程计
    void onTimer();

    // 里程计积分（统一的中间航向角积分）
    void IntegrateImuOnce(const sensor_msgs::msg::Imu& imu);

    void IntegrateOdomOnce(const ChassisFeedback& chassis_data);
    void IntegrateOmniOdomOnce(const ChassisFeedback& chassis_data);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    /**
     * 发布底盘电机反馈（差速 / 全向通用，共享内存直通）。
     *
     * 取代原先借用 ChassisMotorControl 发反馈的做法——那让同一类型同时承担
     * 控制与反馈两个方向，反馈里被迫携带 estop/neutral 这类无对应物的控制位，
     * 而电流、温度、错误码却无处安放。
     *
     * 消费方：① mobile_cmd_omni 取 position/speed 作转向与速度的平滑基准
     *         ② 标定与诊断——NIS 新息检验需要**原始量测**（/feedback/odom 的
     *            twist 是 EKF 更新后的估计值，用它算新息恒等于 0）；轮级打滑
     *            诊断需要 wheel_motor_current
     */
    void PublishMotorFeedback(const ChassisFeedback& chassis_data);

    // 发布里程计与 TF
    void PublishOdometry();

    // 辅助函数
    bool IsImuAvailable();

    // 仅按编码器轮速更新静止标志，供 IMU 静止初始化做交叉确认。
    // 必须独立于 is_stopped_：后者依赖 IMU 角速度，而 IMU 初始化又依赖静止判定，
    // 直接复用会形成循环依赖。本函数无 IMU 依赖，在 onTimer 早期无条件调用。
    void UpdateWheelStoppedFlag(const ChassisFeedback& chassis_data);

    // 底盘反馈失效判定（fail-fast）：反馈持续无效超过 odom_stale_timeout_ 即判失效，
    // 失效期间停发里程计与 TF，只报 ERROR；恢复后自动续发。
    // 判据锚点是「上次 feedback_valid 为真的节点时刻」——PeekLatest 自带 10 ms
    // 新鲜度门（写读同为 CLOCK_MONOTONIC），本函数只在其上加持续时长与状态迁移。
    void UpdateOdomHealth(bool feedback_valid);

    // 切换里程计模式：派生两个布尔；进/出 EKF 时灌初值/接续协方差；切换航向源时清 IMU 缓存
    void ApplyOdomMode(OdomMode new_mode);

    // 读取机器人模型 差速/全向
    void LoadChassisConfig(const std::filesystem::path& cfg_path);

    // 把当前 IMU 相关参数整体下发给 ImuProcessor（热更新各分支共用，避免漏传新参数）
    void ApplyImuProcessorParams();

private:
    // 参数
    double wheel_track_;      // 轮距 b (m)
    std::string odom_frame_;  // "odom"
    std::string base_frame_;  // "base_footprint"
    double odom_rate_hz_;     // 发布频率

    // 发布器与 TF
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr refined_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorFeedback>::SharedPtr motor_feedback_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    std::unique_ptr<ImuProcessor> imu_processor_;
    std::unique_ptr<ChassisEkf> ekf_;

    // 里程计模式（参数 odom_mode，四选一）。下面两个布尔由它派生，供既有分支复用，勿直接改。
    OdomMode odom_mode_ = OdomMode::Ekf;
    bool use_imu_yaw_ = true;     // = ModeUsesImu(odom_mode_)
    bool use_ekf_fusion_ = true;  // = ModeUsesEkf(odom_mode_)
    bool is_imu_inited_ = false;
    bool is_aerial_ = false;  // 悬空/打滑检测结果（由 EKF 填写）

    // 底盘模型
    int chassis_model_ = 0;

    // 位姿状态
    double odom_x_ = 0.0;
    double odom_y_ = 0.0;
    double odom_theta_ = 0.0;

    // 底盘数据缓存
    std::shared_ptr<ChassisFeedback> curr_chassis_data_;
    std::shared_ptr<ChassisFeedback> prev_chassis_data_;

    // IMU 数据缓存
    std::shared_ptr<sensor_msgs::msg::Imu> curr_imu_;
    std::shared_ptr<sensor_msgs::msg::Imu> prev_imu_;

    // 运动状态标志（用于控制 IMU 是否积分角度）
    bool is_stopped_ = true;
    // 纯编码器静止标志（IMU 静止初始化的第二信源，无 IMU 依赖）。
    // 初值 true：开机时底盘反馈可能晚于 IMU，而此时机器人确实静止。
    bool is_wheel_stopped_ = true;
    // 全向底盘参数定义（单位：mm，角度：rad）
    static constexpr double CHASSIS_SIZE_L = 460.0;        // 前后轮间距（wheelbase）
    static constexpr double CHASSIS_SIZE_W = 345.0;        // 左右轮间距（track）
    static constexpr double OMNI_CHASSIS_WHEEL_D = 107.0;  // 轮径
    // 差速底盘轮参（标定值，单位：m）
    double wheel_radius_ = 0.0865;     // 轮半径 r（左右相同）
    double wheel_track_eff_ = 0.4390;  // 有效轴距 b_eff（独立于几何轴距）
    // 发布用的速度
    double speed_v_ = 0.0;
    double omega_r_ = 0.0;
    double speed_v_y_ = 0.0;

    // param
    double odom_factor_ = 1.0;
    double imu_factor_ = 1.0;

    // IMU初始化参数（默认值与 declare_parameter / chassis_params.yaml 保持一致）
    int imu_max_buff_size_ = 500;
    double acc_excite_threshold_ = 0.04;
    double gyro_excite_threshold_ = 0.004;
    double imu_reinit_duration_ = 3600.0;
    double gravity_magnitude_ = 9.7887;
    double gyro_bias_warn_threshold_ = 0.01;  // 零偏告警门限，非静止判据
    double acc_magnitude_tolerance_ = 0.2;

    // 死区参数
    double speed_deadzone_ = 5e-3;  // m/s
    double omega_deadzone_ = 5e-3;  // rad/s
    double imu_timeout_ = 0.1;      // s

    // IMU 超时检测
    // 最近一帧 IMU 的**消息时间戳**。用它而非接收时刻，是因为超时要衡量的是
    // 「数据有多旧」——能一并检出消息在订阅队列里积压的情形（接收时刻看不出来，
    // 但拿 200 ms 前的角速度去积分比降级更糟）。
    // 前提是发布方与本节点同时钟源：w1_server 同进程用节点时钟；dexe_sensors/imu
    // 已于 2026-09-01 从 rclcpp::Clock() 改为节点时钟。此前提由 IsImuAvailable
    // 中的 sanity 检查兜底，不靠约定。
    rclcpp::Time last_imu_stamp_{0, 0, RCL_ROS_TIME};

    // —— IMU 来源自动判定 ——
    ImuSource imu_source_ = ImuSource::Unknown;
    rclcpp::Time imu_detect_start_{0, 0, RCL_ROS_TIME};  // 宽限期起点
    uint32_t last_shm_imu_stamp_ = 0;                    // 上一次看到的 shm IMU 时间戳
    double imu_source_detect_grace_ = 5.0;               // 判定宽限期 (s)

    // —— 底盘反馈失效（fail-fast）——
    double odom_stale_timeout_ = 0.5;  // s，持续无效超过即判失效
    bool odom_stale_ = false;          // 当前是否处于失效态
    bool has_feedback_ever_ = false;   // 收到过有效反馈：区分「未初始化」与「运行中失效」
    rclcpp::Time last_feedback_valid_time_{0, 0, RCL_ROS_TIME};  // 上次 feedback_valid 为真的节点时刻
    rclcpp::Time odom_stale_since_{0, 0, RCL_ROS_TIME};          // 本次失效起点
    uint32_t odom_stale_count_ = 0;                              // 累计失效次数
    double odom_stale_total_sec_ = 0.0;                          // 累计失效时长 (s)

    // EKF 参数
    ChassisEkf::Params ekf_params_;
    // 非 EKF 模式的协方差（EKF 退化形态），与 ekf_params_ 共用 Q/R
    DeadReckoningCov dr_cov_;
    bool last_imu_available_ = false;  // 本周期发布的 ω 是否来自 IMU（决定 twist ω 方差取值）

    std::mutex imu_yaw_mutex_;
};

}  // namespace chassis_node
