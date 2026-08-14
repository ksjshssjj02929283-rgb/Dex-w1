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

#include "chassis_interfaces/msg/chassis_motor_control.hpp"
#include "imu_processor.h"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "shared_memory_def.h"

namespace chassis_node {

class ChassisNode : public rclcpp::Node {
public:
    explicit ChassisNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~ChassisNode() override = default;

    // 禁止拷贝与赋值
    ChassisNode(const ChassisNode&) = delete;
    ChassisNode& operator=(const ChassisNode&) = delete;

    void ResetOdometry();

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    void ImuMsgCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // 主计时器：按固定频率积分与发布里程计
    void onTimer();

    // 里程计积分（统一的中间航向角积分）
    void IntegrateImuOnce(const sensor_msgs::msg::Imu& imu);

    void IntegrateOdomOnce(const ChassisFeedback& chassis_data);
    void IntegrateOmniOdomOnce(const ChassisFeedback& chassis_data);

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter>& parameters);

    //  发布全向底盘反馈信息
    void PublishOmniMotorFeedback(const ChassisFeedback& chassis_data);

    // 发布里程计与 TF
    void PublishOdometry();

    // 辅助函数
    bool IsImuAvailable(const rclcpp::Time& curr_time);

    // 读取机器人模型 差速/全向
    void LoadChassisConfig(const std::filesystem::path& cfg_path);

private:
    // 参数
    double wheel_track_;      // 轮距 b (m)
    std::string odom_frame_;  // "odom"
    std::string base_frame_;  // "base_footprint"
    double odom_rate_hz_;     // 发布频率

    // 发布器与 TF
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr refined_imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<chassis_interfaces::msg::ChassisMotorControl>::SharedPtr omni_motor_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;

    std::unique_ptr<ImuProcessor> imu_processor_;

    bool use_imu_yaw_ = true;  // 是否使用IMU航向
    bool is_imu_inited_ = false;

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
    // 全向底盘参数定义（单位：mm，角度：rad）
    static constexpr double CHASSIS_SIZE_L = 460.0;        // 前后轮间距（wheelbase）
    static constexpr double CHASSIS_SIZE_W = 345.0;        // 左右轮间距（track）
    static constexpr double OMNI_CHASSIS_WHEEL_D = 107.0;  // 轮径
    // 差速底盘参数定义 （单位：mm，角度：rad）
    static constexpr double DIFF_CHASSIS_WHEEL_D = 165.3;
    // 发布用的速度
    double speed_v_ = 0.0;
    double omega_r_ = 0.0;
    double speed_v_y_ = 0.0;

    // param
    double odom_factor_ = 1.0;
    double imu_factor_ = 1.0;

    // IMU初始化参数
    int imu_max_buff_size_ = 1500;
    double acc_excite_threshold_ = 0.015;
    double gyro_excite_threshold_ = 0.002;
    double imu_reinit_duration_ = 3600.0;
    double gravity_magnitude_ = 9.7887;

    // 死区参数
    double speed_deadzone_ = 5e-3;  // m/s
    double omega_deadzone_ = 5e-3;  // rad/s
    double imu_timeout_ = 0.1;      // s

    // IMU 超时检测
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};

    std::mutex imu_yaw_mutex_;
};

}  // namespace chassis_node
