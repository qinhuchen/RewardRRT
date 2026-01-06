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

/* Author: Qinhu Chen */

#include <limits>

#include "ompl/geometric/planners/rrt/RewardRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/tools/config/SelfConfig.h"




ompl::geometric::RewardRRT::RewardRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "RewardRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;
    Planner::declareParam<double>("negative_sigma_limit", this, &RewardRRT::setNegativeSigmaLimit,
                            &RewardRRT::getNegativeSigmaLimit, "2:0.1:10.");
    Planner::declareParam<double>("beta", this, &RewardRRT::setBeta, &RewardRRT::getBeta, "1.0:1.0:100.");
    Planner::declareParam<double>("forgetting_factor", this, &RewardRRT::setForgettingFactor, &RewardRRT::getForgettingFactor, "0.:.01:1.");
    Planner::declareParam<double>("range", this, &RewardRRT::setRange, &RewardRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("R_cum", this, &RewardRRT::setR_cum, &RewardRRT::getR_cum, "0.001:0.001:1.");
    Planner::declareParam<double>("R_inc", this, &RewardRRT::set_R_inc, &RewardRRT::get_R_inc, "0.0001:0.0001:1.");
}

ompl::geometric::RewardRRT::~RewardRRT()
{
    freeMemory();
}

void ompl::geometric::RewardRRT::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RewardRRT::clear()
{
    Planner::clear();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<Motion *, Motion *>(nullptr, nullptr);
}

void ompl::geometric::RewardRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    // Configuring the range of the planner
    if (maxDistance_ < std::numeric_limits<double>::epsilon())
    {
        sc.configurePlannerRange(maxDistance_);
        maxDistance_ *= magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    }

    // Configuring nearest neighbors structures for the planning trees
    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                 {
                                     return distanceFunction(a, b);
                                 });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                {
                                    return distanceFunction(a, b);
                                });

    // Setup the optimization objective, if it isn't specified
    if (!pdef_ || !pdef_->hasOptimizationObjective())
    {
        OMPL_INFORM("%s: No optimization objective specified.  Defaulting to mechanical work minimization.",
                    getName().c_str());
        opt_ = std::make_shared<base::MechanicalWorkOptimizationObjective>(si_);
    }
    else
        opt_ = pdef_->getOptimizationObjective();
    
    connectionRange_ = 10.0 * si_->getStateSpace()->getLongestValidSegmentLength();
}


ompl::geometric::RewardRRT::Motion *ompl::geometric::RewardRRT::addMotion(const base::State *state, TreeData &tree,
                                                                    Motion *parent)
{
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, state);
    motion->parent = parent;
    motion->root = parent != nullptr ? parent->root : nullptr;
    // Add start motion to the tree
    tree->add(motion);
    return motion;
}

void ompl::geometric::RewardRRT::PVF(std::vector<double>& reward)
{
    for(std::size_t i=0;i<reward.size();i++)
    {
        reward.at(i)=reward.at(i)*(1-forgetting_factor);
    }

}

int ompl::geometric::RewardRRT::FindMaxReward(const std::vector<double>& vec) 
{
    auto max_element_iter = std::max_element(vec.begin(), vec.end());
    return std::distance(vec.begin(), max_element_iter);
}

unsigned int ompl::geometric::RewardRRT::getDOF()
{
   return si_->getStateSpace()->getDimension();
}

void ompl::geometric::RewardRRT::CompositionState(base::State *com_state, base::State *state1, base::State *state2)
{
    double *jointstate1=state1->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double *jointstate2=state2->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    int dof=getDOF();
    for(int i=0;i<dof;i++)
    {
        com_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]=jointstate2[i]-jointstate1[i];
    }
}

double ompl::geometric::RewardRRT::BiasCalculate(Eigen::Vector2d state)
{
    double x=-weight_reward_cum * state[0] - weight_reward_inc * state[1];
    double bias=1.0-1 / (1 + std::exp(-x));
    if(bias>0.9)bias=0.9;
    return bias;
}

void ompl::geometric::RewardRRT::StartStateUpdate(double reward_inc)
{
    start_measurement_z=reward_inc;
    start_kalman_gain_K = start_state_P_pred * start_measurement_matrix_H * (start_measurement_matrix_H.transpose() * start_state_P_pred * start_measurement_matrix_H ).inverse();
    start_state_x = start_state_x_pred + start_kalman_gain_K * (start_measurement_z - (start_measurement_matrix_H.transpose() * start_state_x_pred));
    start_state_P = (Eigen::Matrix2d::Identity() - start_kalman_gain_K * start_measurement_matrix_H.transpose()) * start_state_P_pred;
    start_state_x_pred = start_state_transition_F * start_state_x;
    start_state_P_pred = start_state_transition_F * start_state_P * start_state_transition_F.transpose() +start_process_noise_Q;
    bias_start=BiasCalculate(start_state_x_pred);
}

void ompl::geometric::RewardRRT::GoalStateUpdate(double reward_inc)
{
    goal_measurement_z=reward_inc;
    goal_kalman_gain_K = goal_state_P_pred * goal_measurement_matrix_H * (goal_measurement_matrix_H.transpose() * goal_state_P_pred *goal_measurement_matrix_H ).inverse();
    goal_state_x = goal_x_state_pred + goal_kalman_gain_K * (goal_measurement_z - goal_measurement_matrix_H.transpose() * goal_x_state_pred);
    goal_state_P = (Eigen::Matrix2d::Identity() - goal_kalman_gain_K * goal_measurement_matrix_H.transpose()) * goal_state_P_pred;
    goal_x_state_pred = goal_state_transition_F * goal_state_x;
    goal_state_P_pred = goal_state_transition_F * goal_state_P * goal_state_transition_F.transpose() +goal_process_noise_Q;
    bias_goal=BiasCalculate(goal_x_state_pred);
}

void ompl::geometric::RewardRRT::StartStatePrint(std::string info_type)
{
    std::cout << "--------------------------"<<info_type<<"--------------------------" << std::endl;
    std::cout << "start_kalman_gain_K = " << start_kalman_gain_K.transpose() << std::endl;
    std::cout << "start State = " << start_state_x.transpose() << std::endl;
    std::cout << "start Covariance = \n" << start_state_P << std::endl;
    std::cout << "start pred State = " << start_state_x_pred.transpose() << std::endl;
    std::cout << "start pred Covariance = \n" << start_state_P_pred << std::endl;
    std::cout << "bias_start = " << bias_start<< std::endl;

}

void ompl::geometric::RewardRRT::GoalStatePrint(std::string info_type)
{
    std::cout << "--------------------------"<<info_type<<"--------------------------" << std::endl;
    std::cout << "goal = " << goal_kalman_gain_K.transpose() << std::endl;
    std::cout << "goal State = " << goal_state_x.transpose() << std::endl;
    std::cout << "goal Covariance = \n" << goal_state_P << std::endl;
    std::cout << "goal pred State = " << goal_x_state_pred.transpose() << std::endl;
    std::cout << "goal pred Covariance = \n" << goal_state_P_pred << std::endl;
    std::cout << "bias_goal = " << bias_goal<< std::endl;
}

void ompl::geometric::RewardRRT::KFInit()
{
    weight_reward_cum=0.7;
    weight_reward_inc=0.3;
    positive_sigma_limit=1.0;
    start_var_R_cum=1.0;
    start_var_R_inc=1.0;
    
    start_q_R_cum=R_cum;
    start_q_R_inc=R_inc;
    start_reward_cum=0;
    start_reward_inc=0;
    start_state_x<<start_reward_cum,start_reward_inc;
    start_state_P<<start_var_R_cum, 1.0,
                        1.0,start_var_R_inc;
    start_state_transition_F << 1, 1, 
             1, 0;
    start_measurement_matrix_H << 0, 1; 
    start_process_noise_Q.diagonal() << start_q_R_cum, start_q_R_inc;

    goal_var_R_cum=1.0;
    goal_var_R_inc=1.0;
    goal_q_R_cum=R_cum;
    goal_q_R_inc=R_inc;
    goal_reward_cum=0;
    goal_reward_inc=0;
    goal_state_x<<goal_reward_cum,goal_reward_inc;
    goal_state_P<<goal_var_R_cum, 1.0, 
                       1.0,goal_var_R_inc;
    goal_state_transition_F << 1,1, 
             1, 0;
    goal_measurement_matrix_H << 0, 1;
    goal_process_noise_Q.diagonal() << goal_q_R_cum, goal_q_R_inc;

    bias_goal=1.0;
    bias_start=1.0;
}

ompl::geometric::RewardRRT::GrowResult ompl::geometric::RewardRRT::extendTree(Motion *nearest, TreeData &tree,
                                                                        Motion *toMotion, Motion *&result, 
                                                                        std::vector<Motion *>& startmotiontree, 
                                                                        std::vector<Motion *>& goalmotiontree,
                                                                        std::vector<double>& start_reward,
                                                                        std::vector<double>& goal_reward)
{
    bool reach = true;

    // Compute the state to extend toward
    bool treeIsStart = (tree == tStart_);
    double d = (treeIsStart ? si_->distance(nearest->state, toMotion->state)
                            : si_->distance(toMotion->state, nearest->state));
    // Truncate the random state to be no more than maxDistance_ from nearest neighbor
    if (d > maxDistance_)
    {
        if (tree == tStart_)
            si_->getStateSpace()->interpolate(nearest->state, toMotion->state, maxDistance_ / d, toMotion->state);
        else
            si_->getStateSpace()->interpolate(toMotion->state, nearest->state, 1.0 - maxDistance_ / d, toMotion->state);
        d = maxDistance_;
        reach = false;
    }

    // Validating the motion
    bool validMotion=(tree == tStart_ ? si_->checkMotion(nearest->state, toMotion->state) :
                           si_->isValid(toMotion->state) && si_->checkMotion(toMotion->state, nearest->state));
    if (validMotion)
    {
        result = addMotion(toMotion->state, tree, nearest);

        Motion *motion_tmp=new Motion(si_);
        si_->copyState(motion_tmp->state,toMotion->state);
        treeIsStart?startmotiontree.push_back(motion_tmp):goalmotiontree.push_back(motion_tmp);
        treeIsStart?PVF(start_reward):PVF(goal_reward);
        treeIsStart?start_reward.push_back(1.0/(si_->distance(toMotion->state, goalmotiontree.at(0)->state))+beta):
        goal_reward.push_back(1.0/(si_->distance(startmotiontree.at(0)->state, toMotion->state))+beta);
        if(treeIsStart)
        {
            start_reward_inc=1.0/(si_->distance(toMotion->state, goalmotiontree.at(0)->state))+beta;
            if(abs(start_reward_inc)>=positive_reward_limit)start_reward_inc=positive_reward_limit;
            StartStateUpdate(start_reward_inc);
            // StartStatePrint("start valid");
        }
        else
        {
            goal_reward_inc=1.0/(si_->distance(goalmotiontree.at(0)->state, toMotion->state))+beta;
            if(abs(goal_reward_inc)>=positive_reward_limit)goal_reward_inc=positive_reward_limit;
            GoalStateUpdate(goal_reward_inc);
            // StartStatePrint("goal valid");
        }
        return reach ? SUCCESS : ADVANCED;
    }
    else
    {
        if(tree == tStart_)
        {
            start_reward_inc=-(1.0/(si_->distance(toMotion->state, goalmotiontree.at(0)->state))+beta);
            if(abs(start_reward_inc)>=negative_reward_limit)start_reward_inc=-negative_reward_limit;
            StartStateUpdate(start_reward_inc);
            // StartStatePrint("start invalid");
        }
        else
        {
            goal_reward_inc=-(1.0/(si_->distance(goalmotiontree.at(0)->state, toMotion->state))+beta);
            if(abs(goal_reward_inc)>=negative_reward_limit)goal_reward_inc=-negative_reward_limit;
            GoalStateUpdate(goal_reward_inc);
            // StartStatePrint("goal invalid");
        }
        return FAILED;
    }
}

ompl::geometric::RewardRRT::GrowResult ompl::geometric::RewardRRT:: extendTree(Motion *toMotion, TreeData &tree, Motion *&result
                                                                        , std::vector<Motion *>& startmotiontree, 
                                                                          std::vector<Motion *>& goalmotiontree,
                                                                          std::vector<double>& start_reward,
                                                                          std::vector<double>& goal_reward)
{
    // Nearest neighbor
    Motion *nearest = tree->nearest(toMotion);
    return extendTree(nearest, tree, toMotion, result, startmotiontree, goalmotiontree, start_reward, goal_reward);
}

bool ompl::geometric::RewardRRT::connectTrees(Motion *nmotion, TreeData &tree, Motion *xmotion, 
                                           std::vector<Motion *>& startmotiontree, std::vector<Motion *>& goalmotiontree,
                                           std::vector<double>& start_reward,std::vector<double>& goal_reward)
{
    // Get the nearest state to nmotion in tree (nmotion is NOT in tree)
    Motion *nearest = tree->nearest(nmotion);
    bool treeIsStart = tree == tStart_;

    // Copy the resulting state into our scratch space
    si_->copyState(xmotion->state, nmotion->state);

    // Do not try to connect states directly.  Must chop up the
    // extension into segments, just in case one piece fails
    // the transition test
    GrowResult result;
    Motion *next = nullptr;
    do
    {
        // Extend tree from nearest toward xmotion
        // Store the result into next
        // This function MAY trash xmotion
        result = extendTree(nearest, tree, xmotion, next, startmotiontree, goalmotiontree, start_reward, goal_reward);

        if (result == ADVANCED)
        {
            nearest = next;

            // xmotion may get trashed during extension, so we reload it here
            si_->copyState(xmotion->state,
                           nmotion->state);  // xmotion may get trashed during extension, so we reload it here
        }
    } while (result == ADVANCED);
    // Successful connection 
    if (result == SUCCESS)
    {
        Motion *startMotion = treeIsStart ? next : nmotion;
        Motion *goalMotion = treeIsStart ? nmotion : next;

        // Make sure start-goal pair is valid
        if (pdef_->getGoal()->isStartGoalPairValid(startMotion->root, goalMotion->root))
        {
            // Since we have connected, nmotion->state and next->state have the same value
            // We need to check one of their parents to avoid a duplicate state in the solution path
            // One of these must be true, since we do not ever attempt to connect start and goal directly.
            if (startMotion->parent != nullptr)
                startMotion = startMotion->parent;
            else
                goalMotion = goalMotion->parent;

            connectionPoint_ = std::make_pair(startMotion, goalMotion);
            return true;
        }
    }

    return false;
}

ompl::base::PlannerStatus ompl::geometric::RewardRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    KFInit();
    OMPL_INFORM("RewardRRT beta = %lf, forgetting_factor = %lf, R_cum = %lf, R_inc = %lf, negative_sigma_limit = %lf", 
        beta, forgetting_factor, R_cum, R_inc, negative_sigma_limit);
    RNG rng_;
    goalBias_=0.7;
    std::vector<double> reward_start;
    std::vector<double> reward_goal;
    std::vector<Motion *> start_motion_tree;
    std::vector<Motion *> goal_motion_tree; 
    reward_start.clear();
    reward_goal.clear();
    reward_start.push_back(0.0);
    reward_goal.push_back(0.0);

    // Basic error checking
    checkValidity();

    // Goal information
    base::Goal *goal = pdef_->getGoal().get();
    auto *gsr = dynamic_cast<base::GoalSampleableRegion *>(goal);

    if (gsr == nullptr)
    {
        OMPL_ERROR("%s: Goal object does not derive from GoalSampleableRegion", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }
    
    // Loop through the (valid) input states and add them to the start tree
    while (const base::State *state = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, state);
        motion->root = motion->state;  // this state is the root of a tree
        auto *start_motion = new Motion(si_);
        si_->copyState(start_motion->state, state);
        start_motion->root=state;
        start_motion_tree.push_back(start_motion);

        // Add start motion to the tree
        tStart_->add(motion);
    
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Start tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Do the same for the goal tree, if it is empty, but only once
    if (tGoal_->size() == 0)
    {
        const base::State *state = pis_.nextGoal(ptc);

        if (state != nullptr)
        {
            Motion *motion = addMotion(state, tGoal_);
            motion->root = motion->state;  // this state is the root of a tree

            auto *goal_motion = new Motion(si_);
            si_->copyState(goal_motion->state, state);
            goal_motion->root=state;
            goal_motion_tree.push_back(goal_motion);

            start_reward_inc=1.0/(si_->distance(start_motion_tree.at(0)->state, goal_motion_tree.at(0)->state))+beta;
            goal_reward_inc=start_reward_inc;
            positive_reward_limit=positive_sigma_limit*start_reward_inc;
            negative_reward_limit=negative_sigma_limit*start_reward_inc;
            start_state_x<<start_reward_cum,start_reward_inc;
            goal_state_x<<goal_reward_cum,goal_reward_inc;
            OMPL_INFORM("start_reward_inc: %lf, goal_reward_inc: %lf",start_reward_inc,goal_reward_inc);

            //KF
            start_state_x_pred = start_state_transition_F * start_state_x;
            start_state_P_pred = start_state_transition_F * start_state_P * start_state_transition_F.transpose() + start_process_noise_Q;
            goal_x_state_pred = goal_state_transition_F * goal_state_x;
            goal_state_P_pred = goal_state_transition_F * goal_state_P * goal_state_transition_F.transpose() + goal_process_noise_Q;
            bias_start=BiasCalculate(start_state_x_pred);
            bias_goal=BiasCalculate(goal_x_state_pred);
            OMPL_INFORM("Initial bias_start = %lf, bias_goal = %lf", bias_start, bias_goal);
        }
    }

    if (tGoal_->size() == 0)
    {
        OMPL_ERROR("%s: Goal tree has no valid states!", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    OMPL_INFORM("%s: Planning started with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    base::StateSamplerPtr sampler = si_->allocStateSampler();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    auto *xmotion = new Motion(si_);
    base::State *xstate = xmotion->state;

    TreeData tree = tStart_;
    TreeData otherTree = tGoal_;

    bool solved = false;
    // Planning loop
    while (!ptc)
    {
        if(start_state_x_pred[0]>goal_x_state_pred[0])
        {
            tree = tGoal_;
            otherTree = tStart_;
            goalBias_=bias_goal;
        }
        else
        {
            tree = tStart_;
            otherTree = tGoal_;
            goalBias_=bias_start;
        }
        // tree==tStart_?OMPL_WARN("Start tree is being expanded") : OMPL_WARN("Goal tree is being expanded");
        if ((gsr != nullptr) && rng_.uniform01() < goalBias_ && gsr->canSample())
        {
            bool treeIsStart = tree == tStart_;
            int index=treeIsStart?FindMaxReward(reward_start):FindMaxReward(reward_goal);
            treeIsStart?CompositionState(rstate,start_motion_tree.at(index)->state,goal_motion_tree.at(0)->state):
            CompositionState(rstate,goal_motion_tree.at(index)->state,start_motion_tree.at(0)->state);
        }
        else
        {
            sampler->sampleUniform(rstate);
        }

        Motion *result; // the motion that gets added in extendTree
        if (extendTree(rmotion, tree, result, start_motion_tree, goal_motion_tree, reward_start, reward_goal) != FAILED)  // we added something new to the tree
        {
            // Try to connect the other tree to the node we just added
            if (connectTrees(result, otherTree, xmotion, start_motion_tree, goal_motion_tree, reward_start, reward_goal))
            {
                // The trees have been connected.  Construct the solution path
                Motion *solution = connectionPoint_.first;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = connectionPoint_.second;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);
                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
        }

    }

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());
    OMPL_INFORM("%s: %u start_reward, %u goal_reward, %u start_motion_tree, %u goal_motion_tree", getName().c_str(), 
    reward_start.size() , reward_goal.size(), start_motion_tree.size(), goal_motion_tree.size());

    reward_start.clear();
    reward_goal.clear();
    for (auto &motion : start_motion_tree)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    for (auto &motion : goal_motion_tree)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
    si_->freeState(rstate);
    si_->freeState(xstate);
    delete rmotion;
    delete xmotion;

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RewardRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);
    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);
    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    if ((connectionPoint_.first != nullptr) && (connectionPoint_.second != nullptr))
        data.addEdge(data.vertexIndex(connectionPoint_.first->state), data.vertexIndex(connectionPoint_.second->state));
}