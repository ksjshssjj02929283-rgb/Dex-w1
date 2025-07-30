#pragma once

#include "data_structure.h"

namespace robot_server_lib
{

    template <int NDoF>
    void fit_cubic_spline(int n, const double dt[], const Eigen::Matrix<double, NDoF, 1> *x, Eigen::Matrix<double, NDoF, 1> *x1,
                          Eigen::Matrix<double, NDoF, 1> *x2)
    {
        Eigen::Matrix<double, NDoF, 1> x1_i = x1[0];
        Eigen::Matrix<double, NDoF, 1> x1_f = x1[n - 1];
        Eigen::Matrix<double, NDoF, 1> *c = x1;
        Eigen::Matrix<double, NDoF, 1> *d = x2;

        c[0].setConstant(0.5);
        d[0] = 3.0 * ((x[1] - x[0]) / dt[0] - x1_i) / dt[0];
        for (int i = 1; i <= n - 2; ++i)
        {
            double dt2 = dt[i - 1] + dt[i];
            double a = dt[i - 1] / dt2;
            Eigen::Matrix<double, NDoF, 1> denom = Eigen::Matrix<double, NDoF, 1>::Constant(2.0) - a * c[i - 1];
            c[i] = Eigen::Matrix<double, NDoF, 1>::Constant(1.0 - a).cwiseQuotient(denom);
            d[i] = 6.0 * ((x[i + 1] - x[i]) / dt[i] - (x[i] - x[i - 1]) / dt[i - 1]) / dt2;
            d[i] = (d[i] - a * d[i - 1]).cwiseQuotient(denom);
        }
        double denom = dt[n - 2] * (2.0 - c[n - 2][0]);
        d[n - 1] = 6.0 * (x1_f - (x[n - 1] - x[n - 2]) / dt[n - 2]);
        d[n - 1] = (d[n - 1] - dt[n - 2] * d[n - 2]) / denom;

        x2[n - 1] = d[n - 1];
        for (int i = n - 2; i >= 0; --i)
            x2[i] = d[i] - c[i].cwiseProduct(x2[i + 1]);

        x1[0] = x1_i;
        for (int i = 1; i < n - 1; ++i)
            x1[i] = (x[i + 1] - x[i]) / dt[i] - (2 * x2[i] + x2[i + 1]) * dt[i] / 6.0;
        x1[n - 1] = x1_f;
    }

    template <int NDoF>
    void init_times(int n, double dt[], const Eigen::Matrix<double, NDoF, 1> *x, const Eigen::Matrix<double, NDoF, 1> &v_max,
                    const Eigen::Matrix<double, NDoF, 1> &v_min)
    {
        for (int i = 0; i < n - 1; ++i)
        {
            double max_time = 0.0;
            for (int j = 0; j < NDoF; ++j)
            {
                double dx = x[i + 1][j] - x[i][j];
                double time = (dx >= 0.0) ? dx / v_max[j] : dx / v_min[j];
                max_time = std::max(max_time, time);
            }
            dt[i] = std::max(dt[i], max_time + std::numeric_limits<double>::epsilon());
        }
    }

    template <int NDoF>
    void apply_global_adjustment(int n, double dt[], JointTrajectory<NDoF> &joint)
    {
        fit_cubic_spline<NDoF>(n, dt, joint.positions.data(), joint.velocities.data(), joint.accelerations.data());

        double tfactor2 = 1.0;
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < NDoF; ++j)
            {
                double v = joint.velocities[i][j];
                double a = joint.accelerations[i][j];
                if (v > joint.max_velocity[j])
                    tfactor2 = std::max(tfactor2, v / joint.max_velocity[j]);
                if (v < joint.min_velocity[j])
                    tfactor2 = std::max(tfactor2, v / joint.min_velocity[j]);
                double acc_factor = (a >= 0.0) ? std::sqrt(std::abs(a / joint.max_acceleration[j])) : std::sqrt(std::abs(a / joint.min_acceleration[j]));
                tfactor2 = std::max(tfactor2, acc_factor);
            }
        }

        for (int i = 0; i < n - 1; ++i)
            dt[i] *= tfactor2;

        fit_cubic_spline<NDoF>(n, dt, joint.positions.data(), joint.velocities.data(), joint.accelerations.data());
    }

    // assert start and end velocities are 0
    template <int NDoF>
    bool parameterizeJointTrajectory(JointTrajectory<NDoF> &joint, std::vector<double> &time_diff, double *total_time)
    {
        int n = joint.positions.size();
        if (n < 4)
            return false;

        joint.velocities.assign(n, Eigen::Matrix<double, NDoF, 1>::Zero());
        joint.accelerations.assign(n, Eigen::Matrix<double, NDoF, 1>::Zero());
        time_diff.assign(n - 1, std::numeric_limits<double>::epsilon());

        init_times(n, time_diff.data(), joint.positions.data(), joint.max_velocity, joint.min_velocity);

        double previous_energy = -1.0;
        while (true)
        {
            std::vector<double> time_factor(n - 1, 1.0);
            bool changed = false;
            fit_cubic_spline<NDoF>(n, time_diff.data(), joint.positions.data(), joint.velocities.data(), joint.accelerations.data());

            double acc_energy = 0.0;
            for (int i = 0; i < n; ++i)
                acc_energy += joint.accelerations[i].squaredNorm();

            if (previous_energy > 0.0)
            {
                if (acc_energy > previous_energy * 1.01)
                {
                    // 可通过额外因子进一步拉长时间，增加平滑性
                    for (int i = 0; i < n - 1; ++i)
                        time_diff[i] *= 1.01;
                }
            }
            previous_energy = acc_energy;

            // check acceleration
            for (int i = 0; i < n; ++i)
            {
                for (int j = 0; j < NDoF; ++j)
                {
                    double a = joint.accelerations[i][j];
                    double factor = 1.0;
                    if (a > joint.max_acceleration[j])
                        factor = std::sqrt(a / joint.max_acceleration[j]);
                    else if (a < joint.min_acceleration[j])
                        factor = std::sqrt(a / joint.min_acceleration[j]);
                    if (factor > 1.01)
                    {
                        changed = true;
                        if (i > 0)
                            time_factor[i - 1] = std::max(time_factor[i - 1], factor);
                        if (i < n - 1)
                            time_factor[i] = std::max(time_factor[i], factor);
                    }
                }
            }
            // check jerk
            for (int i = 0; i < n - 1; ++i)
            {
                Eigen::Matrix<double, NDoF, 1> jerk = (joint.accelerations[i + 1] - joint.accelerations[i]) / time_diff[i];
                for (int j = 0; j < NDoF; ++j)
                {
                    double jerk_j = jerk[j];
                    double factor = 1.0;
                    if (jerk_j > joint.max_jerk[j])
                        factor = std::cbrt(jerk_j / joint.max_jerk[j]);
                    else if (jerk_j < joint.min_jerk[j])
                        factor = std::cbrt(jerk_j / joint.min_jerk[j]);
                    if (factor > 1.01)
                    {
                        changed = true;
                        time_factor[i] = std::max(time_factor[i], factor);
                    }
                }
            }

            if (!changed)
                break;

            for (int i = 0; i < n - 1; ++i)
                time_diff[i] *= time_factor[i];
        }

        // smooth time, comment this to keep time-optimal
        const double ratio = 1.0;
        const double max_time = *std::max_element(time_diff.begin(), time_diff.end());
        int wall = (n - 1) / 10;
        for (int i = 0; i < n - 1; ++i)
        {
            double scale = 1.1;
            if (i < wall)
            {
                scale += 1.0 * (double)(wall - i) * (double)(wall - i) / ((double)(wall) * (double)(wall));
            }
            if (i >= n - 1 - wall)
            {
                scale += 1.0 * (double)(i + 1 - (n - 1 - wall)) * (double)(i + 1 - (n - 1 - wall)) / ((double)(wall) * (double)(wall));
            }
            time_diff[i] = ((1.0 - ratio) * time_diff[i] + ratio * max_time) * scale;
        }

        apply_global_adjustment(n, time_diff.data(), joint);

        double t = 0.0;
        for (const auto &dt : time_diff)
        {
            t += dt;
        }
        *total_time = t;

        return true;
    }
    template <int nDoF>
    struct SplineSegment
    {
        double t_start;                            // 起始时间
        double duration;                           // 区间时长 Δt
        Eigen::Matrix<double, nDoF, 1> a, b, c, d; // 系数向量
    };

    template <int nDoF>
    class TrajectorySpline
    {
    public:
        std::vector<SplineSegment<nDoF>> segments;

        void fromJointTrajectory(const JointTrajectory<nDoF> &joint_trajectory, const std::vector<double> &dt)
        {
            const auto &positions = joint_trajectory.positions;
            const auto &velocities = joint_trajectory.velocities;
            const auto &accelerations = joint_trajectory.accelerations;
            segments.clear();
            double t_now = 0.0;
            for (size_t i = 0; i < dt.size(); ++i)
            {
                SplineSegment<nDoF> seg;
                seg.t_start = t_now;
                seg.duration = dt[i];
                seg.a = positions[i];
                seg.b = velocities[i];
                seg.c = 0.5 * accelerations[i];
                seg.d = (accelerations[i + 1] - accelerations[i]) / (6.0 * dt[i]);
                segments.push_back(seg);
                t_now += dt[i];
            }
        }

        void evaluate(double t, Eigen::Matrix<double, nDoF, 1> &pos, Eigen::Matrix<double, nDoF, 1> &vel, Eigen::Matrix<double, nDoF, 1> &acc) const
        {
            if (segments.empty())
                return;

            const SplineSegment<nDoF> *seg = nullptr;
            for (const auto &s : segments)
            {
                if (t < s.t_start + s.duration)
                {
                    seg = &s;
                    break;
                }
            }

            if (!seg)
                seg = &segments.back();

            double tau = t - seg->t_start;
            tau = std::max(0.0, std::min(tau, seg->duration));

            pos = seg->a + seg->b * tau + seg->c * tau * tau + seg->d * tau * tau * tau;
            vel = seg->b + 2.0 * seg->c * tau + 3.0 * seg->d * tau * tau;
            acc = 2.0 * seg->c + 6.0 * seg->d * tau;
        }

        void evaluate(double t, Eigen::Matrix<double, nDoF, 1> &pos, Eigen::Matrix<double, nDoF, 1> &vel) const
        {
            if (segments.empty())
                return;

            const SplineSegment<nDoF> *seg = nullptr;
            for (const auto &s : segments)
            {
                if (t < s.t_start + s.duration)
                {
                    seg = &s;
                    break;
                }
            }

            if (!seg)
                seg = &segments.back();

            double tau = t - seg->t_start;
            tau = std::max(0.0, std::min(tau, seg->duration));

            pos = seg->a + seg->b * tau + seg->c * tau * tau + seg->d * tau * tau * tau;
            vel = seg->b + 2.0 * seg->c * tau + 3.0 * seg->d * tau * tau;
        }
    };

} // namespace robot_server_lib