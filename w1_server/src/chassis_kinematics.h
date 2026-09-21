#pragma once
// 底盘运动学：几何参数的**唯一来源**，正解与逆解共用同一份 L/W/D。
//
// 改前同一组量在三处各写一遍（`w1_server.h` 用米、`chassis_node.h` 用毫米、nav 侧 planB 又一份），
// 改一处漏两处就会让正逆解互相矛盾——里程计说走了 1 m，控制以为走了 1.05 m，且两边都"自洽"。
//
// 本文件不依赖 ROS、不分配内存、无虚函数，可直接在 1 kHz 实时线程里调用，也可脱离节点单测。
// 数学推导见知识库《四轮舵向底盘正逆运动学原理》。

#include <array>
#include <cmath>

namespace chassis_node {

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

/// 机体系 twist（x 前 / y 左 / z 上，ROS REP-103）
struct Twist2D {
    double vx = 0.0;   // m/s
    double vy = 0.0;   // m/s，差速底盘恒为 0
    double wz = 0.0;   // rad/s
};

/// 底盘几何——**名义值**：图纸或卷尺量出来的物理尺寸，不参与标定。
/// 命令侧（逆解）与里程计侧（正解）共用同一个实例。
struct ChassisGeometry {
    // —— 全向（四舵轮）——
    double omni_wheelbase = 0.460;  // L：前后轮心距 (m)，沿 x
    double omni_track = 0.345;      // W：左右轮心距 (m)，沿 y
    double omni_wheel_d = 0.107;    // 轮径 (m)

    // —— 差速 ——
    double diff_track = 0.4390;         // 轴距 b (m)
    double diff_wheel_radius = 0.0865;  // 轮半径 r (m)
};

/// 底盘标定量——由标定程序写入的**修正项**，与上面的名义几何刻意分开放。
///
/// ⚠️ `odom_factor` 是当前标定方法用的**集总系数**（一个数吃掉所有线尺度误差），比较粗暴，
///    后续可能被更具体的参数取代，候选方向：
///      · 逐轮轮径修正（全向 4 个 / 差速 2 个）——能分离左右轮径不等造成的弧线漂移；
///      · 舵角零位偏置（全向 4 个）—— **已落地**，见 ChassisCalibration::omni_steer_zero_rad；
///      · 角尺度单列（有效轴距）——现在靠 imu_factor 与 odom_factor 间接吸收。
///    换成具体参数时**只需改本结构与 ChassisKinematics 的换算函数**，几何、调用点、
///    上层命令通路都不用动——这正是把标定量从几何里拆出来的目的。
struct ChassisCalibration {
    /// 编码器线尺度。正解与逆解**对称**使用：
    ///   正解（里程计）：上报速度 = 名义换算 × odom_factor —— 报出真实走了多少；
    ///   逆解（命令）  ：下发转速 = 名义换算 ÷ odom_factor —— 让实际跑出期望速度。
    /// 只在一侧用会造成"里程准了、命令偏了"，且两边各自自洽、不易察觉。
    double odom_factor = 1.0;

    /// 舵角零位偏置 (rad)，四轮各一个，轮序同 ChassisKinematics::OmniWheelIndex。
    /// 定义：轮子真实几何朝向 = 舵角编码器读数 + omni_steer_zero_rad[i]。
    ///
    /// 与 odom_factor 一样**对称**使用，方向相反：
    ///   正解（里程计）：真实朝向 = 编码器反馈 + zero —— 让里程计看到轮子实际朝哪；
    ///   逆解（命令）  ：编码器目标 = 理想朝向 − zero —— 把轮子真正摆到理想朝向。
    /// 只补一侧会造成"控制对了里程计错"（或反之），且两边各自自洽——与 odom_factor
    /// 只在一侧用是同构的失败模式，单测 SteerZeroAppliesSymmetricallyToBothDirections 锁住。
    ///
    /// 零位不齐的后果不只是里程计不准：四轮速度矢量互不平行，底盘被合力矩推着转
    /// （前后轴零位差 1° ⇒ 直行 10 m 偏航约 22°），残差由轮胎侧向变形吸收 ⇒ 摩地、电机过载。
    /// 全 0 时与引入本参数前逐位一致。
    std::array<double, 4> omni_steer_zero_rad{};
};

/// 纯计算的运动学求解器。拷贝廉价（只含 POD），按值传递即可。
class ChassisKinematics {
public:
    static constexpr int kOmniWheels = 4;
    /// 轮序：0 前左 / 1 前右 / 2 后左 / 3 后右（与控制侧 wheelLongitudinalRow 一致）
    enum OmniWheelIndex { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };

    ChassisKinematics() = default;
    explicit ChassisKinematics(const ChassisGeometry& g, const ChassisCalibration& c = {}) : g_(g), c_(c) {}

    void SetGeometry(const ChassisGeometry& g) { g_ = g; }
    void SetCalibration(const ChassisCalibration& c) { c_ = c; }
    const ChassisGeometry& geometry() const { return g_; }
    const ChassisCalibration& calibration() const { return c_; }

    /// 第 i 轮在机体系的位置 r_i
    Vec2 OmniWheelPosition(int i) const {
        const double x = (i == kFrontLeft || i == kFrontRight) ? g_.omni_wheelbase * 0.5 : -g_.omni_wheelbase * 0.5;
        const double y = (i == kFrontLeft || i == kRearLeft) ? g_.omni_track * 0.5 : -g_.omni_track * 0.5;
        return {x, y};
    }

    /// 逆解：刚体速度场 v_i = v + ω × r_i ⇒ v_ix = vx − ω·y_i，v_iy = vy + ω·x_i
    void OmniInverse(const Twist2D& t, std::array<Vec2, kOmniWheels>& out) const {
        for (int i = 0; i < kOmniWheels; ++i) {
            const Vec2 r = OmniWheelPosition(i);
            out[i].x = t.vx - t.wz * r.y;
            out[i].y = t.vy + t.wz * r.x;
        }
    }

    /// 正解：8 个观测解 3 个未知量（超定）。
    /// 这里沿用现网实现的等权平均——两条基线给出的角速度估计各占一半。
    /// 最小二乘应按基线平方加权（L² : W²），噪声低约 4%，属后续优化项，改动时请同步里程计标定。
    Twist2D OmniForward(const std::array<double, kOmniWheels>& speed_mps,
                        const std::array<double, kOmniWheels>& angle_rad) const {
        std::array<double, kOmniWheels> vx{};
        std::array<double, kOmniWheels> vy{};
        double sx = 0.0;
        double sy = 0.0;
        for (int i = 0; i < kOmniWheels; ++i) {
            // 用**真实朝向**而非编码器读数投影：零位不齐时二者差 δ_i，
            // 直行工况下编码器读数恒为 0 ⇒ sin(0)=0 ⇒ w_base 支路整条失效，
            // 合成角速度只剩真值的一半。补偿后两条基线才都还原出真实 ω。
            const double ang = OmniSteerTrueFromFb(i, angle_rad[i]);
            vx[i] = speed_mps[i] * std::cos(ang);
            vy[i] = speed_mps[i] * std::sin(ang);
            sx += vx[i];
            sy += vy[i];
        }
        // 左右基线（用 x 分量）与前后基线（用 y 分量）各给一个独立的 ω 估计
        const double w_track = (vx[kFrontRight] + vx[kRearRight] - vx[kFrontLeft] - vx[kRearLeft]) /
                               (2.0 * g_.omni_track);
        const double w_base = (vy[kFrontLeft] + vy[kFrontRight] - vy[kRearLeft] - vy[kRearRight]) /
                              (2.0 * g_.omni_wheelbase);
        return {0.25 * sx, 0.25 * sy, 0.5 * (w_track + w_base)};
    }

    /// 舵角零位换算——与下面的 rpm 换算同属"标定量出现的地方"，调用方不自己加减。
    ///
    /// ⚠️ 两个方向**不能只实现一个**：正解补偿让里程计看到真实朝向，逆解补偿让轮子真正
    ///    摆到理想朝向；只做一侧时两边各自自洽却互相矛盾，是最难查的一类问题。
    ///
    /// ⚠️ 逆解侧要在 atan2 出理想朝向后**立即**换到编码器坐标系，此后机械限位、择近解、
    ///    对齐限速全部沿用原逻辑即可——因为目标角与反馈角此时同在编码器坐标系。
    ///    反过来把反馈换到真实朝向再比较，机械限位会用错坐标系（限位约束的是舵机行程）。

    /// 编码器读数 → 轮子真实朝向（正解侧）
    double OmniSteerTrueFromFb(int i, double fb_rad) const { return fb_rad + c_.omni_steer_zero_rad[i]; }
    /// 理想真实朝向 → 应下发的编码器目标（逆解侧）
    double OmniSteerCmdFromTrue(int i, double true_rad) const { return true_rad - c_.omni_steer_zero_rad[i]; }

    /// rpm ↔ m/s：**标定量只在这几个换算函数里出现**，调用方一律不再自己乘。
    /// 将来 odom_factor 换成逐轮修正时，也只改这里。
    double OmniSpeedToRpm(double mps) const { return mps * 60.0 / (M_PI * g_.omni_wheel_d * c_.odom_factor); }
    double OmniRpmToSpeed(double rpm) const { return rpm * M_PI * g_.omni_wheel_d * c_.odom_factor / 60.0; }

    /// 差速逆解：v_r = vx + ω·b/2，v_l = vx − ω·b/2
    void DiffInverse(const Twist2D& t, double* left_rpm, double* right_rpm) const {
        const double half = g_.diff_track * 0.5;
        *right_rpm = DiffSpeedToRpm(t.vx + t.wz * half);
        *left_rpm = DiffSpeedToRpm(t.vx - t.wz * half);
    }

    /// 差速正解
    Twist2D DiffForward(double left_mps, double right_mps) const {
        return {0.5 * (left_mps + right_mps), 0.0, (right_mps - left_mps) / g_.diff_track};
    }

    double DiffRpmToSpeed(double rpm) const {
        return rpm * (2.0 * M_PI / 60.0) * g_.diff_wheel_radius * c_.odom_factor;
    }
    double DiffSpeedToRpm(double mps) const {
        return mps / ((2.0 * M_PI / 60.0) * g_.diff_wheel_radius * c_.odom_factor);
    }

private:
    ChassisGeometry g_{};
    ChassisCalibration c_{};
};

}  // namespace chassis_node
