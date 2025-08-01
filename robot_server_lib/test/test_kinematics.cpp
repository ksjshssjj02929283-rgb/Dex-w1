#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <chrono>

#include "kinematics_include.h"
#include "utilities.h"

using namespace robot_server_lib;

static constexpr std::array<double, 7> q_min = {-2.9670597283903604, -2.0943951023931953, -2.9670597283903604, -2.356194490192345,
                                                -2.9670597283903604, -1.5707963267948966, -2.9670597283903604};
static constexpr std::array<double, 7> q_max = {2.9670597283903604, 1.5707963267948966, 2.9670597283903604, 1.5707963267948966,
                                                2.96705972839036041, 1.5707963267948966, 2.9670597283903604};

Eigen::Matrix<double, 7, 1> randomJointWithinLimit()
{
  static std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

  Eigen::Matrix<double, 7, 1> q;
  for (size_t i = 0; i < q.size(); ++i)
  {
    std::uniform_real_distribution<double> dist(q_min[i], q_max[i]);
    q[i] = dist(rng);
  }
  return q;
}

Eigen::Matrix<double, 7, 1> randomDisturbance()
{
  static std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());

  Eigen::Matrix<double, 7, 1> q;
  for (size_t i = 0; i < q.size(); ++i)
  {
    std::uniform_real_distribution<double> dist(-0.2, 0.2);
    q[i] = dist(rng);
  }
  return q;
}

// int main(int argc, char **argv)
TEST(KinematicsTest, Compare)
{
  std::filesystem::create_directories("log/");
  nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server", 10);
  nanolog::set_log_level(nanolog::LogLevel::INFO);

  kinematics::SRSKinematics srs = kinematics::SRSKinematics(kinematics::W1LeftArmIndustrial());
  kinematics::YaikKinematics yaik(0);

  Eigen::VectorXd q(7), pose(7);
  q << -0.45, -0.65, 0.35, -1.5, -1, 1.1, -0.4;
  pose << -0.163635, 0.363727, 0.262755, -0.229399, -0.440151, 0.599072, 0.628296;
  auto poseQ = PoseQuaternion::FromVector7d(pose);
  poseQ.position(0) += 0.1;

  auto start_time0 = std::chrono::high_resolution_clock::now();
  auto pose0 = srs.GetFK(q);
  auto end_time0 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> time0 = end_time0 - start_time0;

  auto start_time1 = std::chrono::high_resolution_clock::now();
  auto pose1 = yaik.GetFK(q);
  auto end_time1 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> time1 = end_time1 - start_time1;

  Eigen::VectorXd q2(q);
  auto start_time2 = std::chrono::high_resolution_clock::now();
  auto ok2 = srs.GetIK(poseQ, q2);
  auto end_time2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> time2 = end_time2 - start_time2;

  Eigen::VectorXd q3(q);
  auto start_time3 = std::chrono::high_resolution_clock::now();
  auto ok3 = yaik.GetIK(poseQ, q3);
  auto end_time3 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> time3 = end_time3 - start_time3;

  std::cout << "FK results:\n"
            << "\tSRS :  " << pose0.ToString() << "  Time: " << time0.count() / 1000.0 << "ms\n"
            << "\tYAIK:  " << pose1.ToString() << "  Time: " << time1.count() / 1000.0 << "ms\n"
            << "IK results:\n"
            << "\tSRS :  Joint: " << q2.transpose() << "  " << srs.GetFK(q2).ToString() << "  Time: " << time2.count() / 1000.0 << "ms\n"
            << "\tYAIK:  Joint: " << q3.transpose() << "  " << yaik.GetFK(q3).ToString() << "  Time: " << time3.count() / 1000.0 << "ms\n";
}

TEST(KinematicsTest, Efficiency)
{
  std::filesystem::create_directories("log/");
  nanolog::initialize(nanolog::GuaranteedLogger(), "log/", "robot_server", 10);
  nanolog::set_log_level(nanolog::LogLevel::INFO);

  kinematics::SRSKinematics srs = kinematics::SRSKinematics(kinematics::W1LeftArmIndustrial());
  kinematics::YaikKinematics yaik(0);

  int count = 0;
  Eigen::Matrix<double, 7, 1> qrd, disturbance;
  PoseQuaternion pose0, pose1;
  double fk_srs = 0.0, fk_yaik = 0.0, ik_srs = 0.0, ik_yaik = 0.0;
  int success_srs = 0, success_yaik = 0;
  while (count < 10000)
  {
    qrd = randomJointWithinLimit();
    disturbance = randomDisturbance();
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      pose0 = srs.GetFK(qrd);
      auto end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::micro> dt = end_time - start_time;
      fk_srs += dt.count() / 1000.0;
    }
    {
      auto start_time = std::chrono::high_resolution_clock::now();
      pose1 = yaik.GetFK(qrd);
      auto end_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::micro> dt = end_time - start_time;
      fk_yaik += dt.count() / 1000.0;
    }
    {
      Eigen::VectorXd qin(qrd + disturbance);
      auto start_time = std::chrono::high_resolution_clock::now();
      auto ok = srs.GetIK(pose0, qin);
      auto end_time = std::chrono::high_resolution_clock::now();
      if (ok)
      {
        success_srs++;
        std::chrono::duration<double, std::micro> dt = end_time - start_time;
        ik_srs += dt.count() / 1000.0;
      }
    }
    {
      Eigen::VectorXd qin(qrd + disturbance);
      auto start_time = std::chrono::high_resolution_clock::now();
      auto ok = yaik.GetIK(pose1, qin);
      auto end_time = std::chrono::high_resolution_clock::now();
      if (ok)
      {
        success_yaik++;
        std::chrono::duration<double, std::micro> dt = end_time - start_time;
        ik_yaik += dt.count() / 1000.0;
      }
    }
    count++;
  }

  std::cout << "Summary:\n"
            << "\tSRS:\n"
            << "\t  FK Average Time: " << fk_srs / 10000.0 << " ms\n"
            << "\t  IK Success Rate: " << (double)success_srs / 100.0 << "%\n"
            << "\t  IK Average Time: " << ik_srs / 10000.0 << " ms\n"
            << "\tYAIK:\n"
            << "\t  FK Average Time: " << fk_yaik / 10000.0 << " ms\n"
            << "\t  IK Success Rate: " << (double)success_yaik / 100.0 << "%\n"
            << "\t  IK Average Time: " << ik_yaik / 10000.0 << " ms\n";
}