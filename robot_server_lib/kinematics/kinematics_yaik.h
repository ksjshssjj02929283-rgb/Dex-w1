#pragma once

#include "kinematics_base.h"
#include "w1_left_arm_industrial_ik_generated.hpp"
#include "w1_right_arm_industrial_ik_generated.hpp"
#include <unordered_set>

namespace robot_server_lib
{
  namespace kinematics
  {

    class YaikKinematics : public KinematicsBase
    {
    public:
      static constexpr int ndof_ = 7;
      static constexpr int aux_input_index = 4;
      using Jps = Eigen::Matrix<double, ndof_, 1>;
      using JpsArr = std::array<double, ndof_>;
      struct YaikKinematicsParams
      {
        // trajectory config
        std::size_t milestones_per_knot; // = n_milestones / n_knots, >= 1
        std::size_t n_knots;             // number of knots
        int dist_type;                   // norm type of distance, -1 defaults to L∞ Norm
        Jps dist_weight;
        double jps_dist_tolerance; // tol of jps-distance between milestones
        // dfs config
        bool update_nodes;      // if a node changes with its previous node
        int dfs_max_iterations; // maximum number of dfs iterations
        // sampling config
        int n_theta;
        double d_theta;

        YaikKinematicsParams()
            : milestones_per_knot(10),
              n_knots(20),
              dist_type(-1),
              dist_weight(Jps::Constant(1.0)),
              jps_dist_tolerance(0.2),
              update_nodes(true),
              dfs_max_iterations(5000),
              n_theta(4),
              d_theta(3.14 / 24.0)
        {
        }

        double GetDistance(const Jps &a, const Jps &b) const
        {
          if (dist_type <= 0)
            return ((a - b).cwiseProduct(dist_weight)).array().abs().maxCoeff();
          else
            return std::pow(((a - b).cwiseProduct(dist_weight)).array().abs().pow(dist_type).sum(), 1.0 / dist_type);
        }

        void SortByDistance(const Jps &q_ref, std::vector<Jps> &jps_list) const
        {
          std::sort(jps_list.begin(), jps_list.end(),
                    [this, q_ref](const Jps &a, const Jps &b)
                    {
                      return this->GetDistance(q_ref, a) > this->GetDistance(q_ref, b);
                    });
        }

        void SampleTheta(const double theta_ref, std::vector<double> &theta_list) const
        {
          theta_list.reserve(2 * n_theta + 1);
          double theta_;
          bool upper_bound_reached = false, lower_bound_reached = false;
          for (int i = n_theta; i > 0; i--)
          {
            theta_ = theta_ref + double(i) * d_theta;
            if (theta_ > 3.14159)
            {
              if (!upper_bound_reached)
              {
                upper_bound_reached = true;
                theta_list.push_back(3.14159);
              }
            }
            else
              theta_list.push_back(theta_);

            theta_ = theta_ref - double(i) * d_theta;
            if (theta_ < -3.14159)
            {
              if (!lower_bound_reached)
              {
                lower_bound_reached = true;
                theta_list.push_back(-3.14159);
              }
            }
            else
              theta_list.push_back(theta_);
          }
          theta_list.push_back(theta_ref);
        }
      };

      YaikKinematics(bool arm_type, YaikKinematicsParams _params = YaikKinematicsParams())
          : params(std::move(_params))
      {
        if (arm_type == 0)
          ik_solver_ptr = std::make_shared<yaik_cpp::W1LeftArmIndustrial_IkSolver>();
        else
          ik_solver_ptr = std::make_shared<yaik_cpp::W1RightArmIndustrial_IkSolver>();
      }

    public:
      PoseQuaternion GetFK(const Eigen::VectorXd &q_in,
                           const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
      {
        assert(q_in.size() == ndof_);
        auto ee_pose = ik_solver_ptr->computeFK(q_in);
        return PoseQuaternion(ee_pose * tcp_frame.toHomogeneousMatrix());
      }

      bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out,
                 const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
      {
        std::vector<Jps> ik_results;
        // GetRawIK(T_ee, q_in_and_out(aux_input_index), ik_results);
        GetIKwithProjectionAndSampling(_toMatrix4d(pose_in), q_in_and_out, ik_results, tcp_frame);
        Jps closest_ik;
        bool ik_get = GetClosestIK(ik_results, q_in_and_out, closest_ik);
        q_in_and_out = closest_ik;
        return ik_get;
      }

      void GetRawIK(const Matrix4d &T_ee, const double aux_input, std::vector<Jps> &ik_results)
      {
        ik_solver_ptr->computeIK(T_ee, aux_input, ik_results);
      }

      // Find closest ik result to q_ref, and the found min_dist should be smaller than *dist_in
      bool GetClosestIK(const std::vector<Jps> &ik_results, const Jps &q_ref, Jps &closest_ik, double *dist_in = nullptr)
      {
        double min_dist = dist_in ? *dist_in : std::numeric_limits<double>::max();
        int bestIndex = -1;
        for (int i = 0; i < ik_results.size(); ++i)
        {
          double dist = GetDistance(q_ref, ik_results[i]);
          if (dist < min_dist)
          {
            min_dist = dist;
            bestIndex = i;
          }
        }
        if (bestIndex >= 0)
        {
          closest_ik = ik_results[bestIndex];
          if (dist_in)
            *dist_in = min_dist;
          return true;
        }
        return false;
      }

      bool ProjectToPose(const Jps &q_in, const Matrix4d &T_ee, Jps &q_projected)
      {
        bool project_ok = ik_solver_ptr->projectToPose(q_in, T_ee, q_projected);
        return project_ok;
      }

      void GetIKwithProjectionAndSampling(const Matrix4d &T_ee, const Jps &q_in, std::vector<Jps> &results, const pinocchio::SE3 &tcp_frame)
      {
        Matrix4d T_ee_corrected = T_ee * _Inverse(tcp_frame.toHomogeneousMatrix());
        Jps q_ref;
        bool project_ok = ProjectToPose(q_in, T_ee_corrected, q_ref);
        if (!project_ok)
          q_ref = q_in;

        std::vector<double> theta_list;
        SampleTheta(q_ref(aux_input_index), theta_list);

        results.clear();
        results.reserve(8 * theta_list.size());
        std::vector<Jps> ik_results;
        for (auto &theta : theta_list)
        {
          GetRawIK(T_ee_corrected, theta, ik_results);
          results.insert(results.end(), ik_results.begin(), ik_results.end());
        }
        // SortByDistance(q_in, results);
      }

      // ee_trajectory should begin with fk(q_start)
      bool RunGlobalRedundancyResolutionForJointTrajectory(const Jps &q_start, const PoseTrajectory &ee_trajectory,
                                                           std::vector<Jps> &output, const pinocchio::SE3 &tcp_frame)
      {
        struct LayerNodes
        {
          std::vector<Jps> &nodes() { return jps_collection; }
          void initIt() { r_iterator = jps_collection.rbegin(); }
          bool empty() { return r_iterator == jps_collection.rend(); }
          const Jps &back() { return *r_iterator; }
          void pop_back() { ++r_iterator; }

        private:
          std::vector<Jps> jps_collection;
          std::vector<Jps>::const_reverse_iterator r_iterator;
        };

        int n_knots = params.n_knots;
        double ee_trajectory_length = ee_trajectory.totalLength();
        if (ee_trajectory_length < 0.01)
        {
          return false;
        }
        else if (ee_trajectory_length < 0.2)
        {
          n_knots = 5;
        }
        else if (ee_trajectory_length < 0.4)
        {
          n_knots = 10;
        }

        std::vector<LayerNodes> graph_nodes; // tree of nodes for DFS
        graph_nodes.resize(n_knots + 1);
        graph_nodes[0].nodes() = {q_start};
        graph_nodes[0].initIt();

        // DFS initialization
        const double knots_phase_gap = 1.0 / double(n_knots);
        int current_knot = 0;
        Matrix4d T_ee = ee_trajectory(0.0 + knots_phase_gap);
        GetIKwithProjectionAndSampling(T_ee, q_start, graph_nodes[current_knot + 1].nodes(), tcp_frame);
        SortByDistance(q_start, graph_nodes[current_knot + 1].nodes());
        graph_nodes[current_knot + 1].initIt();
        current_knot++;

        int iter_count = 0;
        std::vector<Jps> local_trajectory, global_trajectory;
        global_trajectory.emplace_back(q_start); // start with q_start
        // start DFS loop
        while (iter_count < params.dfs_max_iterations)
        {
          // backtrack judgement
          while (graph_nodes[current_knot].empty())
          {
            if (current_knot == 1)
            {
              LOG_CRIT << "Global redundancy resolution error: no ik result at all\n";
              return false;
            }
            current_knot--;
            graph_nodes[current_knot].pop_back();
            if (!params.update_nodes && graph_nodes[current_knot].empty())
            {
              LOG_CRIT << "Global redundancy resolution error: no ik result\n";
              return false;
            }
          }
          iter_count++;
          // connect
          const Jps &parent_jps = graph_nodes[current_knot - 1].back();
          const Jps &current_jps = graph_nodes[current_knot].back();
          const double current_phase = double(current_knot) / double(n_knots);
          bool connected =
              Connect(ee_trajectory, parent_jps, current_phase - knots_phase_gap,
                      current_jps, current_phase, local_trajectory, tcp_frame);
          if (connected)
          {
            global_trajectory.resize((current_knot - 1) * params.milestones_per_knot + 1);
            global_trajectory.insert(global_trajectory.end(), local_trajectory.begin(),
                                     local_trajectory.end());
            if (current_knot == n_knots)
            {
              output = std::move(global_trajectory);
              return true;
            }
            // expand
            if (params.update_nodes || graph_nodes[current_knot + 1].nodes().empty())
            {
              T_ee = ee_trajectory(current_phase + knots_phase_gap);
              GetIKwithProjectionAndSampling(T_ee, current_jps, graph_nodes[current_knot + 1].nodes(), tcp_frame);
            }
            SortByDistance(current_jps, graph_nodes[current_knot + 1].nodes());
            graph_nodes[current_knot + 1].initIt();
            current_knot++;
          }
          else
          {
            // skip
            graph_nodes[current_knot].pop_back();
          }
        }
        LOG_CRIT << "Global redundancy resoltion error: reached max iters\n";
        return false;
      }

      double GetDistance(const Jps &a, const Jps &b) const
      {
        return params.GetDistance(a, b);
      }

      void SortByDistance(const Jps &q_ref, std::vector<Jps> &jps_list) const
      {
        params.SortByDistance(q_ref, jps_list);
      }

      void SampleTheta(const double theta_ref, std::vector<double> &theta_list) const
      {
        params.SampleTheta(theta_ref, theta_list);
      }

      bool Connect(const PoseTrajectory &trajectory, const Jps &jps1, const double phase1, const Jps &jps2,
                   const double phase2, std::vector<Jps> &local_trajectory, const pinocchio::SE3 &tcp_frame)
      {
        int n_waypoints = params.milestones_per_knot;
        // preliminary judgement
        if (GetDistance(jps1, jps2) > n_waypoints * params.jps_dist_tolerance)
          return false;

        local_trajectory.clear();
        local_trajectory.reserve(n_waypoints);
        for (int i = 0; i < n_waypoints; ++i)
        {
          const double local_phase = double(i + 1) / double(n_waypoints);
          Jps closest_ik_res;
          if (i != n_waypoints - 1)
          {
            Jps jps_ = jps1 * (1 - local_phase) + jps2 * local_phase;
            double phase_ = phase1 * (1 - local_phase) + phase2 * local_phase;
            Matrix4d T_ee = trajectory(phase_) * _Inverse(tcp_frame.toHomogeneousMatrix());

            std::vector<Jps> ik_results;
            GetRawIK(T_ee, jps_(aux_input_index), ik_results);
            bool ik_get = GetClosestIK(ik_results, jps_, closest_ik_res);
            if (!ik_get)
              return false; // equals to ik_results.empty()
          }
          else
            closest_ik_res = jps2;

          const Jps &jps_last = (i == 0) ? jps1 : local_trajectory.back();
          // connect the adjacent milestones
          if (GetDistance(closest_ik_res, jps_last) > params.jps_dist_tolerance)
          {
            local_trajectory.clear();
            return false;
          }
          local_trajectory.push_back(std::move(closest_ik_res));
        }
        return true;
      };

    private:
      Matrix4d _toMatrix4d(const PoseQuaternion &pose_in)
      {
        Matrix4d T_ee;
        T_ee << pose_in.orientation.toRotationMatrix(), pose_in.position,
            0, 0, 0, 1;
        return T_ee;
      }

      Matrix4d _Inverse(const Matrix4d &mat4d)
      {
        Matrix4d result;
        result << mat4d.topLeftCorner<3, 3>().transpose(), -mat4d.topLeftCorner<3, 3>().transpose() * mat4d.topRightCorner<3, 1>(), 0, 0, 0, 1;
        return result;
      }

      YaikKinematicsParams params;
      std::shared_ptr<yaik_cpp::IkSolverBase<7>> ik_solver_ptr;
    };

  } // namespace kinematics
} // namespace robot_server_lib