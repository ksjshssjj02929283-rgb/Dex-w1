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

#include "dexe_rp/collision/CoalCollision.h"
#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/planner/StateToPoseBase.h"
#include "dexe_rp/planner/StateValidatorBase.h"

namespace DexeRP {
namespace Planner {
class CoalCollisionValidator final : public StateValidatorBase {
public:
    /**
     * @brief Construct a new CoalCollisionValidator object
     *
     * @param coal_collision The CoalCollision instance used for collision checking. The collision object in the
     * CoalCollision instance should be properly initialized before passing in.
     * @param state_to_pose The StateToPoseBase instance used for converting the input state to the corresponding link
     * poses. The state_to_pose instance should be properly initialized before passing in, e.g., the state joint names
     * and fixed joint positions should be set if needed.
     *
     * @note The constructor will make a mapping which will map the robot links poses(in order of the state_to_pose's
     * GetFullJointOrder()) to the corresponding coal collision components and buffers.
     */
    CoalCollisionValidator(std::shared_ptr<DexeRP::Collision::CoalCollision> coal_collision,
                           std::shared_ptr<DexeRP::Planner::StateToPoseBase> state_to_pose)
        : coal_collision_(std::move(coal_collision)), state_to_pose_(std::move(state_to_pose)) {
        const std::vector<std::string> full_active_joint_names_order = state_to_pose_->GetFullJointOrder();
        std::unordered_map<std::string, size_t> component_name_to_buffer_index;
        std::vector<size_t> component_link_counts;

        pose_index_to_binding_.resize(full_active_joint_names_order.size());

        for (size_t pose_idx = 0; pose_idx < full_active_joint_names_order.size(); ++pose_idx) {
            std::string component_name;
            size_t link_index_in_component;
            if (!coal_collision_->FindLinkInComponent(
                    state_to_pose_->GetChildLinkNameFromJointName(full_active_joint_names_order[pose_idx]),
                    component_name, link_index_in_component)) {
                DEXERP_ERROR("Link '{}' not found in any component.", full_active_joint_names_order[pose_idx]);
                // return;
                throw std::runtime_error(fmt::format("Link '{}' not found in any coal robot component.",
                                                     full_active_joint_names_order[pose_idx]));
            }
            size_t component_idx = 0;
            auto it = component_name_to_buffer_index.find(component_name);
            if (it == component_name_to_buffer_index.end()) {
                component_idx = component_names_.size();
                component_name_to_buffer_index[component_name] = component_idx;
                component_names_.push_back(component_name);
                component_link_counts.push_back(link_index_in_component + 1);
            } else {
                component_idx = it->second;
                component_link_counts[component_idx] =
                    std::max(component_link_counts[component_idx], link_index_in_component + 1);
            }

            pose_index_to_binding_[pose_idx] = {component_idx, link_index_in_component};
        }

        component_pose_buffers_.resize(component_names_.size());
        for (size_t i = 0; i < component_names_.size(); ++i) {
            component_pose_buffers_[i].resize(component_link_counts[i]);
        }
    }

    ~CoalCollisionValidator() override = default;

    bool IsStateValid(const CVecXd& state) override;

    std::shared_ptr<DexeRP::Collision::CoalCollision> GetCoalCollision() { return coal_collision_; }
    std::shared_ptr<const DexeRP::Collision::CoalCollision> GetCoalCollision() const { return coal_collision_; }

private:
    std::mutex mutex_;

    std::shared_ptr<DexeRP::Collision::CoalCollision> coal_collision_;
    std::shared_ptr<DexeRP::Planner::StateToPoseBase> state_to_pose_;

    struct LinkBinding {
        size_t component_buffer_index;
        size_t link_index_in_component;
    };

    std::vector<std::string> component_names_;
    std::vector<std::vector<Mat4d>> component_pose_buffers_;
    std::vector<LinkBinding> pose_index_to_binding_;
};
}  // namespace Planner
}  // namespace DexeRP