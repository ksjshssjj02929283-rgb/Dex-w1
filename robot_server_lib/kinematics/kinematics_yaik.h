#pragma once

#include "kinematics_base.h"
#include "yaik/w1_left_indistrial/w1_left_arm_industrial_ik_generated.h"
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
        int dist_type{-1};
        Jps dist_weight{Jps::Constant(1.0)};
        int n_theta{4};
        double d_theta{3.14 / 24.0};

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
          theta_list.clear();
          theta_list.reserve(2 * n_theta + 1);
          std::unordered_set<double> inserted;
          for (int i = n_theta; i > 0; --i)
          {
            double theta_plus = theta_ref + i * d_theta;
            double theta_minus = theta_ref - i * d_theta;
            theta_plus = std::clamp(theta_plus, -3.14159, 3.14159);
            theta_minus = std::clamp(theta_minus, -3.14159, 3.14159);
            if (inserted.insert(theta_plus).second)
              theta_list.push_back(theta_plus);
            if (inserted.insert(theta_minus).second)
              theta_list.push_back(theta_minus);
          }
          theta_list.push_back(theta_ref);
        }
      };

      YaikKinematics(const YaikKinematicsParams &params)
          : _params(params)
      {
        ik_solver_ptr = std::make_shared<yaik_cpp::w1_left_arm_industrial_ik>();
      }

    public:
      PoseQuaternion GetFK(const Eigen::VectorXd &q_in,
                           const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
      {
        assert(q_in.size() == ndof_);
        JpsArr q_arr;
        _VecToArr(q_in, q_arr);
        auto ee_pose = ik_solver_ptr->computeFK(q_arr);
        return PoseQuaternion(ee_pose * tcp_frame.toHomogeneousMatrix());
      }

      bool GetIK(const PoseQuaternion &pose_in, Eigen::VectorXd &q_in_and_out,
                 const pinocchio::SE3 &tcp_frame = pinocchio::SE3::Identity()) override
      {
        Matrix4d T_ee = _GetTranformMatrix4d(pose_in, tcp_frame);
        std::vector<Jps> ik_results;
        GetRawIK(T_ee, q_in_and_out(aux_input_index), ik_results);
        Jps closest_ik;
        bool ik_get = GetClosestIK(ik_results, q_in_and_out, closest_ik);
        q_in_and_out = closest_ik;
        return ik_get;
      }

      void GetRawIK(const Matrix4d &T_ee, const double theta, std::vector<Jps> &ik_results)
      {
        auto ik_results_in_arr = ik_solver_ptr->computeIK(T_ee, theta);
        ik_results.clear();
        for (auto ik_arr : ik_results_in_arr)
        {
          ik_results.push_back(_ArrToVec(ik_arr));
        }
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

      void ProjectToConstraint(const Jps &q_in, const Matrix4d &T_ee, Jps &q_projected)
      {
        JpsArr temp;
        _VecToArr(q_in, temp);
        yaik_cpp::numericalRefinement<ndof_>(ik_solver_ptr->computeFK, ik_solver_ptr->computeTwistJacobian, T_ee, temp);
        _ArrToVec(temp, q_projected);
      }

      void GetIKwithProjectionAndSampling(const Matrix4d &T_ee, const Jps &q_in, std::vector<Jps> &results)
      {
        Jps q_ref;
        ProjectToConstraint(q_in, T_ee, q_ref);

        std::vector<double> theta_list;
        SampleTheta(q_ref(aux_input_index), theta_list);

        results.clear();
        results.reserve(8 * theta_list.size());
        std::vector<Jps> ik_results;
        for (auto &theta : theta_list)
        {
          GetRawIK(T_ee, theta, ik_results);
          results.insert(results.end(), ik_results.begin(), ik_results.end());
        }
        // SortByDistance(q_in, results);
      }

      double GetDistance(const Jps &a, const Jps &b) const
      {
        return _params.GetDistance(a, b);
      }

      void SortByDistance(const Jps &q_ref, std::vector<Jps> &jps_list) const
      {
        _params.SortByDistance(q_ref, jps_list);
      }

      void SampleTheta(const double theta_ref, std::vector<double> &theta_list) const
      {
        _params.SampleTheta(theta_ref, theta_list);
      }

    private:
      void _ArrToVec(const JpsArr &arr, Jps &vec)
      {
        vec = Eigen::Map<const Jps>(arr.data());
      }
      Jps _ArrToVec(const JpsArr &arr)
      {
        return Eigen::Map<const Jps>(arr.data());
      }

      void _VecToArr(const Jps &vec, JpsArr &arr)
      {
        std::copy(vec.data(), vec.data() + ndof_, arr.begin());
      }

      Matrix4d _GetTranformMatrix4d(const PoseQuaternion &pose_in, const pinocchio::SE3 &tcp_frame)
      {
        auto rotation_mat = pose_in.orientation.toRotationMatrix();
        Matrix4d T_ee;
        T_ee << rotation_mat * tcp_frame.rotation().transpose(),
            pose_in.position - rotation_mat * tcp_frame.rotation().transpose() * tcp_frame.translation(),
            0, 0, 0, 1;
        return T_ee;
      }

      const YaikKinematicsParams _params;

      std::shared_ptr<yaik_cpp::w1_left_arm_industrial_ik> ik_solver_ptr;
    };

  } // namespace kinematics
} // namespace robot_server_lib