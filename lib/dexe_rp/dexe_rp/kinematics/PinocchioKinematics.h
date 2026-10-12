/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <casadi/casadi.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <unordered_set>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/Types.h"

namespace DexeRP {
namespace Kinematics {
class PinocchioKinematics {
public:
    struct JointMapping {
        std::string joint_name_;
        size_t input_index_;
        size_t model_index_;
    };

    /**
     * @note URDF file should not contain floating or planar joints.
     */
    PinocchioKinematics(const std::string& urdf_path);
    ~PinocchioKinematics() = default;

    /**
     * @brief Set the joint mappings between input joint order and Pinocchio model joint order.
     * This function is necessary when the input joint order is different from the order of joints in the Pinocchio
     * model.
     *
     * @param input_joint_names The vector of joint names in the order of the input qpos.
     * - The size of this vector should match the number of joints in the model.
     * - The order of elements in this vector should correspond to the order of joint values in the input qpos vector.
     *
     * @note The joint names in the input vector should exactly match the joint names in the Pinocchio model. The
     * function will throw an error if any joint name is not found in the model.
     */
    bool SetJointMappings(const std::vector<std::string>& input_joint_names) {
        joint_mappings_.clear();
        if (input_joint_names.size() != model_.njoints - 1) {
            DEXERP_ERROR(
                "Input joint names size {} does not match model njoints {}. Please provide a valid joint names vector.",
                input_joint_names.size(), model_.njoints - 1);
            return false;
        }
        for (size_t idx = 0; idx < model_.njoints - 1; ++idx) {
            const std::string& joint_name = model_.names[idx + 1];
            auto it = std::find(input_joint_names.begin(), input_joint_names.end(), joint_name);
            if (it == input_joint_names.end()) {
                DEXERP_ERROR(
                    "Joint name '{}' not found in input joint names. Please provide a valid joint names vector.",
                    joint_name);
                return false;
            }
            size_t input_index = std::distance(input_joint_names.begin(), it);
            JointMapping mapping{joint_name, input_index, idx};
            joint_mappings_.emplace(joint_name, mapping);
        }
        return true;
    }

    /**
     * @brief Convert the input qpos vector to the model qpos vector based on the joint mappings.
     *
     * @param input_qpos The input qpos vector in the order of the input joint names.
     * @param model_qpos The output qpos vector in the order of the Pinocchio model joints.
     *
     * @note This function should be called after setting the joint mappings using SetJointMappings(). The size of the
     * input qpos vector should match the size of the joint mappings.
     */
    bool ConvertInputQposToModelQpos(const CVecXd& input_qpos, CVecXd& model_qpos) {
        if (joint_mappings_.empty()) {
            DEXERP_ERROR("Joint mappings are not set. Please call SetJointMappings() before converting qpos.");
            return false;
        }
        if (input_qpos.size() != joint_mappings_.size()) {
            DEXERP_ERROR(
                "Input qpos size {} does not match joint mappings size {}. Please provide a valid input qpos vector.",
                input_qpos.size(), joint_mappings_.size());
            return false;
        }
        model_qpos.resize(model_.nq);
        for (const auto& [joint_name, mapping] : joint_mappings_) {
            model_qpos[mapping.model_index_] = input_qpos[mapping.input_index_];
        }
        return true;
    }

    /**
     * @brief Convert the input qpos map to the model qpos vector based on the joint mappings.
     *
     * @param input_qpos_map The input qpos map with joint names as keys and joint values as values.
     * @param model_qpos The output qpos vector in the order of the Pinocchio model joints.
     *
     * @note This function doesn't require the joint mappings to be set, as it directly uses the joint names to find the
     * corresponding values.
     */
    bool ConvertInputQposToModelQpos(const std::unordered_map<std::string, double>& input_qpos_map,
                                     CVecXd& model_qpos) {
        if (input_qpos_map.size() != model_.nq) {
            DEXERP_ERROR("Input qpos map size {} does not match model nq {}. Please provide a valid input qpos map.",
                         input_qpos_map.size(), model_.nq);
            return false;
        }
        model_qpos.resize(model_.nq);
        for (size_t idx = 0; idx < model_.nq; ++idx) {
            const std::string& joint_name = model_.names[idx + 1];
            auto it = input_qpos_map.find(joint_name);
            if (it == input_qpos_map.end()) {
                DEXERP_ERROR("Joint name '{}' not found in input qpos map. Please provide a valid input qpos map.",
                             joint_name);
                return false;
            }
            model_qpos[idx] = it->second;
        }
        return true;
    }

    /**
     * @brief Compute the forward kinematics of the robot links based on the input qpos.
     *
     * @param qpos_map The input qpos map with joint names as keys and joint values as values.
     * @param joint_poses The output map with joint names as keys and their corresponding poses, in order of the
     * Pinocchio model joints.
     */
    bool ComputeForwardKinematics(const std::unordered_map<std::string, double>& qpos_map,
                                  std::map<std::string, Mat4d>& joint_poses);

    /**
     * @brief Compute the forward kinematics of the robot links based on the input qpos.
     *
     * @param qpos The input qpos vector in the order of the Pinocchio model joints.
     * @param joint_poses The output vector of joint poses in the order of the Pinocchio model joints.
     */
    bool ComputeForwardKinematics(const CVecXd& qpos, std::vector<Mat4d>& joint_poses);

    /**
     * @brief Compute the forward kinematics of a specific frame based on the input qpos.
     *
     * @param frame_name The name of the frame to compute the forward kinematics for.
     * @param qpos The input qpos vector in the order of the Pinocchio model joints.
     * @param frame_pose The output pose of the specified frame.
     */
    bool ComputeFrameForwardKinematics(const std::string& frame_name, const CVecXd& qpos, Mat4d& frame_pose);

    /**
     * @brief Check if a joint name exists in the model.
     */
    bool CheckJointNameExistence(const std::string& name) const { return model_.existJointName(name); }

    /**
     * @brief Check if a frame name exists in the model.
     */
    bool CheckFrameNameExistence(const std::string& name) const { return model_.existFrame(name); }

    /**
     * @brief Get the full list of active joint names in the model, in the order of the Pinocchio model joints.
     */
    const std::vector<std::string> GetFullActiveJointNames() const {
        std::vector<std::string> full_active_joint_names;
        for (size_t i = 1; i < model_.names.size(); ++i) {
            full_active_joint_names.push_back(model_.names[i]);
        }
        return full_active_joint_names;
    }

    /**
     * @brief Get the child link name corresponding to a given joint name.
     *
     * @note In URDF files, the joint usually attaches on the child link.
     */
    const std::string GetChildLinkNameFromJointName(const std::string& joint_name) {
        if (!model_.existJointName(joint_name)) {
            DEXERP_ERROR("Joint name '{}' does not exist in the model. Please provide a valid joint name.", joint_name);
            return "";
        }

        const pinocchio::JointIndex joint_index = model_.getJointId(joint_name);

        for (const auto& frame : model_.frames) {
            if (frame.parentJoint == joint_index && frame.type == pinocchio::BODY) {
                return frame.name;
            }
        }

        return "";
    }

private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    std::unordered_map<std::string, JointMapping> joint_mappings_;
};

}  // namespace Kinematics
}  // namespace DexeRP
