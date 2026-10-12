/*
 * ----------------------------------------------------------------------------
 * Copyright (c) 2021-2026 DexForce Technology Co., Ltd.
 *
 * All rights reserved.
 * ----------------------------------------------------------------------------
 */

#pragma once

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <vector>

#include "dexe_rp/common/CommonHeader.h"
#include "dexe_rp/planner/StateValidatorBase.h"

namespace DexeRP {
namespace Planner {
// using namespace ompl::base;
// using namespace ompl::geometric;
class OMPLPlanner {
public:
    /**
     * @brief Construct a new OMPLPlanner object
     *
     * @param state_dim The dimension of the state space, typically the number of joints for robot motion planning
     * @param state_lower_bounds The lower bounds of the state space
     * @param state_upper_bounds The upper bounds of the state space
     * @param state_validator_ptr A pointer to the state validator used for checking the validity of states during
     * planning
     * @param checking_resolution The resolution for state validity checking
     * @param sample_range The range for sampling new states during planning, typically used in RRT-based planners.
     */
    OMPLPlanner(const size_t state_dim,
                const CVecXd& state_lower_bounds,
                const CVecXd& state_upper_bounds,
                std::shared_ptr<DexeRP::Planner::StateValidatorBase> state_validator_ptr,
                const double checking_resolution = 0.005,
                const double sample_range = 0.1);
    ~OMPLPlanner() = default;

    /**
     * @brief Set the state space bounds. This function can be used to update the state space bounds after the planner
     * is constructed.
     */
    bool SetStateBounds(const CVecXd& state_lower_bounds, const CVecXd& state_upper_bounds);

    /**
     * @brief Set the planning time limit for the planner. This function can be used to update the planning time limit
     * after the planner is constructed.
     */
    bool SetPlanningTimeLimit(const double time_limit) {
        if (time_limit <= 0) {
            DEXERP_ERROR("Planning time limit must be positive. Provided time limit: {}.", time_limit);
            return false;
        }
        planning_time_limit_ = time_limit;
        return true;
    }

    /**
     * @brief Set the state validity checking resolution. This function can be used to update the checking resolution
     * after the planner is constructed.
     */
    bool SetCheckingResolution(const double checking_resolution) {
        if (checking_resolution <= 0) {
            DEXERP_ERROR("Checking resolution must be positive. Provided checking resolution: {}.",
                         checking_resolution);
            return false;
        }
        checking_resolution_ = checking_resolution;
        return true;
    }

    /**
     * @brief Set the sample range for sampling new states during planning. This function can be used to update the
     * sample range after the planner is constructed.
     */
    bool SetSampleRange(const double sample_range) {
        if (sample_range <= 0) {
            DEXERP_ERROR("Sample range must be positive. Provided sample range: {}.", sample_range);
            return false;
        }
        sample_range_ = sample_range;
        return true;
    }

    /**
     * @brief Check the validity of the given state using the provided state validator. This function will be used as
     * the state validity checker in the OMPL planner.
     */
    bool IsStateValid(const ompl::base::State* state) {
        if (state == nullptr) {
            DEXERP_ERROR("Input state pointer is null.");
            return false;
        }
        if (state_validator_ptr_ == nullptr) {
            DEXERP_ERROR("State validator pointer is null.");
            return false;
        }

        const auto* real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();

        CVecXd q(state_dim_);
        for (size_t i = 0; i < state_dim_; ++i) {
            q[i] = real_state->values[i];
        }

        bool is_valid = state_validator_ptr_->IsStateValid(q);

        return is_valid;
    }

    /**
     * @brief Plan a path from the start state to the goal state. The planned path will be returned as a vector of
     * states in the state space. The start and goal states should be of the same dimension as the state space dimension
     * specified in the constructor.
     */
    bool Plan(const CVecXd& start_state, const CVecXd& goal_state, std::vector<CVecXd>& path);

private:
    size_t state_dim_;
    std::shared_ptr<ompl::base::RealVectorStateSpace> state_space_;
    ompl::base::RealVectorBounds state_bounds_;
    std::shared_ptr<DexeRP::Planner::StateValidatorBase> state_validator_ptr_;
    double planning_time_limit_{5.0};
    double checking_resolution_{0.005};
    double sample_range_{0.1};
};
}  // namespace Planner
}  // namespace DexeRP
