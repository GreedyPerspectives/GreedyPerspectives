/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_SOLVER_H
#define VISUALGROUPCOVERAGEPLANNER_SOLVER_H

#include "Model.h"
#include <AIToolbox/MDP/Algorithms/ValueIteration.hpp>
#include <AIToolbox/MDP/Policies/Policy.hpp>

class SubmodularSolver
{
public:
    SubmodularSolver(std::vector<DiscreteAgent> partitions,
                     std::shared_ptr<DiscreteContext> context,
                     model::single_agent::AgentRewardFunction rewardFunction,
                     int iterations = 1, bool collisionConstraints = true);

    std::map<AgentId, std::vector<State>> solveSequential();

    std::vector<double> _rewards;

private:
    std::vector<DiscreteAgent> _partitions;
    std::shared_ptr<DiscreteContext> _context;
    // TODO switch to being just DiscreteAgent and state
    model::single_agent::AgentRewardFunction _rewardFunction;
    int _iterations; // Round Robin planning when greater than 1
    bool _collisionConstraints;
};

#endif // VISUALGROUPCOVERAGEPLANNER_SOLVER_H