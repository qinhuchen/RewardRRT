/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025, Northwestern Polytechnical University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Northwestern Polytechnical University
*     nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: qinhu chen */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RewardRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RewardRRT_

#include <cmath>
#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/OptimizationObjective.h"
#include <Eigen/Dense>
#include <fstream>
#include <iostream>

namespace ompl
{
    namespace geometric
    {
        class RewardRRT : public base::Planner
        {
            private:
            double weight_reward_cum;
            double weight_reward_inc;
            double bias_start;
            double bias_goal;

            double start_var_R_cum;
            double start_var_R_inc;
            double start_q_R_cum;
            double start_q_R_inc;
            Eigen::Vector2d start_state_x; 
            Eigen::Matrix2d start_state_P; 
            Eigen::Vector2d start_state_x_pred;
            Eigen::Matrix2d start_state_P_pred;
            double start_measurement_z;
            Eigen::Vector2d start_kalman_gain_K;
            Eigen::Matrix2d start_state_transition_F;
            Eigen::Vector2d start_measurement_matrix_H;
            Eigen::Matrix2d start_process_noise_Q;

            double goal_var_R_cum;
            double goal_var_R_inc;
            double goal_q_R_cum;
            double goal_q_R_inc;
            Eigen::Vector2d goal_state_x; 
            Eigen::Matrix2d goal_state_P;
            Eigen::Vector2d goal_x_state_pred;
            Eigen::Matrix2d goal_state_P_pred;
            double goal_measurement_z;
            Eigen::Vector2d goal_kalman_gain_K;
            Eigen::Matrix2d goal_state_transition_F;
            Eigen::Vector2d goal_measurement_matrix_H;
            Eigen::Matrix2d goal_process_noise_Q;

            double positive_sigma_limit;
            double negative_sigma_limit;
            double positive_reward_limit;
            double negative_reward_limit;
            double start_reward_cum;
            double start_reward_inc;
            double goal_reward_cum;
            double goal_reward_inc;
            double goalBias_; 

            double beta;
            double forgetting_factor;
            double R_cum;
            double R_inc;

        public:
            unsigned int getDOF();
            void PVF(std::vector<double>& reward);
            int FindMaxReward(const std::vector<double>& vec);
            void CompositionState(base::State *com_state, base::State *state1, base::State *state2);
            double BiasCalculate(Eigen::Vector2d state);
            void StartStatePrint(std::string info_type);
            void GoalStatePrint(std::string info_type);
            void KFInit();
            void StartStateUpdate(double reward_inc);
            void GoalStateUpdate(double reward_inc);

            /// Constructor
            RewardRRT(const base::SpaceInformationPtr &si);
            ~RewardRRT() override;
            void clear() override;
            void setup() override;
            void getPlannerData(base::PlannerData &data) const override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /// \brief Set the maximum possible length of any one motion in
            ///  the search tree.  Very short/long motions may inhibit
            
            void setR_cum(double R_cum)
            {
                this->R_cum = R_cum;
            }

            double getR_cum() const
            {
                return this->R_cum;
            }

            void set_R_inc(double R_inc)
            {
                this->R_inc = R_inc;
            }

            double get_R_inc() const
            {
                return this->R_inc;
            }

            void setNegativeSigmaLimit(double limit)
            {
                negative_sigma_limit = limit;
            }

            double getNegativeSigmaLimit() const
            {
                return negative_sigma_limit;
            }

            /// @brief Set the forgetting_factor the planner is using
            /// @param forgetting_factor
            void setForgettingFactor(double forgetting_factor)
            {
                this->forgetting_factor = forgetting_factor;
            }

            /// \brief Get the forgetting_factor the planner is using
            double getForgettingFactor() const
            {
                return forgetting_factor;
            }

            /// \brief Set the beta the planner is using
            void setBeta(double beta)
            {
                this->beta = beta;
            }

            /// \brief Get the beta the planner is using
            double getBeta() const
            {
                return beta;
            }

            ///  the exploratory capabilities of the planner.
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /// \brief Get the range the planner is using
            double getRange() const
            {
                return maxDistance_;
            }

            /// \brief Set a different nearest neighbors datastructure
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            /// \brief Representation of a motion in the search tree
            class Motion
            {
            public:
                /// \brief Default constructor
                Motion() = default;

                /// \brief Constructor that allocates memory for the state
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                /// \brief The state contained by the motion
                base::State *state{nullptr};

                /// \brief The parent motion in the exploration tree
                Motion *parent{nullptr};

                /// \brief Pointer to the root of the tree this motion is
                /// contained in.
                const base::State *root{nullptr};
            };

            /// \brief Free all memory allocated during planning
            void freeMemory();

            /// \brief The nearest-neighbors data structure that contains the
            /// entire the tree of motions generated during planning.
            using TreeData = std::shared_ptr<NearestNeighbors<Motion *>>;

            /// \brief Add a state to the given tree.  The motion created
            /// is returned.
            Motion *addMotion(const base::State *state, TreeData &tree, Motion *parent = nullptr);

            /// \brief The result of a call to extendTree
            enum GrowResult
            {
                /// No extension was possible
                FAILED,
                /// Progress was made toward extension
                ADVANCED,
                /// The desired state was reached during extension
                SUCCESS
            };

            GrowResult extendTree(Motion *toMotion, TreeData &tree, Motion *&result, 
                                  std::vector<Motion *>& startmotiontree, std::vector<Motion *>& goalmotiontree,
                                  std::vector<double>& start_reward,std::vector<double>& goal_reward);

            /// \brief Extend \e tree from \e nearest toward \e toMotion.
            /// Store the result of the extension, if any, in result
            GrowResult extendTree(Motion *nearest, TreeData &tree, Motion *toMotion, Motion *&result
                                , std::vector<Motion *>& startmotiontree, std::vector<Motion *>& goalmotiontree,
                                std::vector<double>& start_reward,std::vector<double>& goal_reward);

            /// \brief Attempt to connect \e tree to \e nmotion, which is in
            /// the other tree.  \e xmotion is scratch space and will be overwritten
            bool connectTrees(Motion *nmotion, TreeData &tree, Motion *xmotion, 
                              std::vector<Motion *>& startmotiontree, std::vector<Motion *>& goalmotiontree,
                              std::vector<double>& start_reward,std::vector<double>& goal_reward);


            /// \brief Compute distance between motions (actually distance between contained states)
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /// \brief The maximum length of a motion to be added to a tree
            double maxDistance_{0.};

            /// \brief The range at which the algorithm will attempt to connect
            /// the two trees.
            double connectionRange_;

            /// \brief The most recent connection point for the two trees.
            /// Used for PlannerData computation.
            std::pair<Motion *, Motion *> connectionPoint_{nullptr, nullptr};

            /// \brief The start tree
            TreeData tStart_;

            /// \brief The goal tree
            TreeData tGoal_;

            ompl::base::OptimizationObjectivePtr opt_;
        };

    }
}

#endif