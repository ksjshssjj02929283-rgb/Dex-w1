/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/kinematics/PinocchioKinematics.h"
#include "dexe_rp/planner/StateToPoseBase.h"

namespace DexeRP {
namespace Planner {
class PinocchioStateToPose final : public StateToPoseBase {
public:
    PinocchioStateToPose(const std::string& urdf_path) : pin_kine_(urdf_path) {}
    ~PinocchioStateToPose() override = default;

    /**
     * @brief Get the full joint(corresponding links) poses in the order of pinocchio model joints' order from the given
     * state
     *
     * @note The output poses is full body poses in the order of the pinocchio robot model. The order of the robot model
     * is provided by GetFullJointOrder() API. The state validator will deal with the mapping of different orders.
     */
    bool StateToPose(const CVecXd& state, std::vector<Mat4d>& poses) override {
        if (state_joint_names_.empty()) {
            DEXERP_ERROR("State joint names are not set. Please call SetStateJointNames() before using StateToPose().");
            return false;
        }
        if (state.size() != state_joint_names_.size()) {
            DEXERP_ERROR("State size {} does not match state joint names size {}. Please provide a valid state vector.",
                         state.size(), state_joint_names_.size());
            return false;
        }
        if (fixed_joint_name_position_map_.size() != fixed_joint_names_.size()) {
            DEXERP_ERROR(
                "Fixed joint name-position map size {} does not match fixed joint names size {}. Please provide a "
                "valid fixed joint name-position map.",
                fixed_joint_name_position_map_.size(), fixed_joint_names_.size());
            return false;
        }
        // state_joint_name_position_map_.clear();
        // for (size_t i = 0; i < state_joint_names_.size(); ++i) {
        //     state_joint_name_position_map_[state_joint_names_[i]] = state[i];
        // }

        // std::unordered_map<std::string, double> full_joint_name_position_map = fixed_joint_name_position_map_;
        // for (const auto& [joint_name, pos] : state_joint_name_position_map_) {
        //     full_joint_name_position_map[joint_name] = pos;
        // }
        CVecXd full_qpos(state_joint_names_.size() + fixed_joint_names_.size());
        for (size_t i = 0; i < full_qpos.size(); ++i) {
            if (i < state_joint_names_.size()) {
                full_qpos[i] = state[i];
            } else {
                full_qpos[i] = fixed_joint_positions_[i - state_joint_names_.size()];
            }
        }

        CVecXd model_qpos;
        bool convert_res = pin_kine_.ConvertInputQposToModelQpos(full_qpos, model_qpos);
        if (!convert_res) {
            DEXERP_ERROR("Failed to convert state qpos to full model qpos.");
            return false;
        }

        poses.clear();
        bool fk_res = pin_kine_.ComputeForwardKinematics(model_qpos, poses);
        if (!fk_res) {
            DEXERP_ERROR("Failed to compute forward kinematics.");
            return false;
        }
        return true;
    }

    /**
     * @brief Get the full joint order of the pinocchio robot model
     */
    const std::vector<std::string> GetFullJointOrder() const override {
        const std::vector<std::string> full_active_joint_names_order = pin_kine_.GetFullActiveJointNames();
        return full_active_joint_names_order;
    }

    /**
     * @brief Get the child link name corresponding to the given joint name
     *
     * @note Typically the poses are used for links, so we get the child link name corresponding to the given joint
     * name.
     */
    const std::string GetChildLinkNameFromJointName(const std::string& joint_name) override {
        return pin_kine_.GetChildLinkNameFromJointName(joint_name);
    }

    /**
     * @brief Set the state joint names in the order of the input state vector. This function should be called before
     * using StateToPose() API.
     */
    bool SetStateJointNames(const std::vector<std::string>& state_joint_names) {
        state_joint_names_.clear();
        fixed_joint_names_.clear();
        for (const auto& name : state_joint_names) {
            if (!pin_kine_.CheckJointNameExistence(name)) {
                DEXERP_ERROR("State joint name '{}' does not exist in the robot model.", name);
                return false;
            }
            state_joint_names_.push_back(name);
        }
        for (const auto& name : pin_kine_.GetFullActiveJointNames()) {
            if (std::find(state_joint_names_.begin(), state_joint_names_.end(), name) == state_joint_names_.end()) {
                fixed_joint_names_.push_back(name);
            }
        }
        fixed_joint_positions_.resize(fixed_joint_names_.size(), 0.0);

        std::vector<std::string> full_input_names = state_joint_names_;
        full_input_names.insert(full_input_names.end(), fixed_joint_names_.begin(), fixed_joint_names_.end());

        bool set_mapping_res = pin_kine_.SetJointMappings(full_input_names);
        if (!set_mapping_res) {
            DEXERP_ERROR("Failed to set joint mappings in PinocchioKinematics.");
            return false;
        }
        return true;
    }

    /**
     * @brief Set the fixed joint positions for the joints that are not included in the state joint names. This function
     * should be called after setting the state joint names and before using StateToPose() API
     */
    bool SetFixedJointPositions(const std::unordered_map<std::string, double>& fixed_joint_name_position_map) {
        fixed_joint_name_position_map_.clear();
        for (const auto& [joint_name, position] : fixed_joint_name_position_map) {
            auto it = std::find(fixed_joint_names_.begin(), fixed_joint_names_.end(), joint_name);
            if (it == fixed_joint_names_.end()) {
                DEXERP_ERROR("Fixed joint name '{}' is not in the list of fixed joints.", joint_name);
                return false;
            }
            fixed_joint_positions_[std::distance(fixed_joint_names_.begin(), it)] = position;

            fixed_joint_name_position_map_[joint_name] = position;
        }
        return true;
    }

private:
    DexeRP::Kinematics::PinocchioKinematics pin_kine_;
    std::vector<std::string> state_joint_names_;
    std::vector<std::string> fixed_joint_names_;
    std::vector<double> fixed_joint_positions_;
    std::unordered_map<std::string, double> state_joint_name_position_map_;
    std::unordered_map<std::string, double> fixed_joint_name_position_map_;
};
}  // namespace Planner
}  // namespace DexeRP