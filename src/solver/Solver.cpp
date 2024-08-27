
/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Solver.h"

SubmodularSolver::SubmodularSolver(
    std::vector<DiscreteAgent> partitions, std::shared_ptr<DiscreteContext> context,
    model::single_agent::AgentRewardFunction rewardFunction, int iterations,
    bool collisionConstraints)
  : _partitions(partitions)
  , _context(context)
  , _rewardFunction(rewardFunction)
  , _iterations(iterations)
  , _collisionConstraints(collisionConstraints)
{}

std::map<AgentId, std::vector<State>> SubmodularSolver::solveSequential()
{
    std::vector<std::map<AgentId, double>> rewards(_partitions[0]._horizon);
    for(int i = 0; i < _iterations; i++)
    {
        if(_iterations > 1)
        {
            spdlog::info("Round Robin Planning Iteration {}", i);
        }
        // Sequentially plan for each agent
        for(auto & agent : _partitions)
        {
            if(i > 0)
            {
                // Remove agent's trajectory and coverage
                _context->unsetTrajectory(agent._id);
            }
            spdlog::info("Agent [{}], Single Agent Planning", agent._id);

            // Remove the state state of the agent we are planning for from the obstacle
            // map

            if(_collisionConstraints)
                _context->_obstacleMap(agent._startState[0], agent._startState[1]) =
                    false;
            auto model = model::single_agent::computeMDP(
                agent, _rewardFunction, _collisionConstraints);

            spdlog::info("Agent [{}], MDP created", agent._id);

            // TODO This should be 1 but the value iteration is a bit wrong
            AIToolbox::MDP::ValueIteration solver(50);
            // Solve to get Q and value function
            auto solution = solver(model);
            if(std::get<0>(solution) != 0)
                spdlog::warn("Solution did not converge: {}", std::get<0>(solution));
            else
                spdlog::info("Solution Found");

            AIToolbox::MDP::Policy policy(agent._stateSpace.size(),
                                          agent._actionSpace.size(),
                                          std::get<1>(solution));

            // FOllow the policy to generate a trajectory
            std::vector<State> traj = {agent._startState};
            spdlog::debug("Agent Trajectory");
            spdlog::debug("{}", traj.back());
            for(int t = 1; t < agent._horizon; t++)
            {
                State s = traj.back();
                StateId sId = agent._stateSpace.encodeState(s);
                ActionId aId = policy.sampleAction(sId); // only 1 action is non-zero

                space::Action a = agent._actionSpace.decodeAction(aId);
                if(a == space::terminalAction)
                    spdlog::warn("Terminal Action Detected");

                space::State sNext = agent._stateSpace.applyAction(traj.back(), a);
                StateId sNextId = agent._stateSpace.encodeState(sNext);
                traj.push_back(sNext);
                double r = 0;
                if((a[0] + a[1]) == 0)
                    r += 100;
                if(a[3] == 0)
                    r += 200;
                rewards[t][agent._id] = r;
                // Checks
                if(model.getTransitionProbability(sId, aId, sNextId) == 0)
                    spdlog::warn(
                        "State transition with probability zero: (SAS), {}:{}, {}:{}, "
                        "{}:{} ",
                        sId,
                        s,
                        aId,
                        a,
                        sNextId,
                        sNext);
                spdlog::debug("{} \t {}", traj.back(), a);
                if(_context->_obstacleMap(traj.back()[0], traj.back()[1]))
                    spdlog::warn("Trajectory Collision Detected at {}", traj.back());
            }

            // Fix the trajectory and add to collision map
            _context->setTrajectory(agent._id, traj);
            double r_s = 0.0;
            for(int t = 0; t < rewards.size(); t++)
            {
                for(auto [fid, ppa] : _context->_totalCoverage[t])
                    r_s += sqrt(ppa);

                for(auto [aId, rew] : rewards[t])
                    r_s += rew;
                spdlog::info("Agent [{}] Time: {}, Reward: {}", agent._id, t, r_s);
            }

            spdlog::info("Agent [{}] Reward: {}", agent._id, r_s);
        }
    }

    std::vector<double> rewardSum(_partitions[0]._horizon, 0);
    for(int t = 0; t < rewards.size(); t++)
    {
        for(auto [fid, ppa] : _context->_totalCoverage[t])
            rewardSum[t] += sqrt(ppa);

        for(auto [aId, rew] : rewards[t])
            rewardSum[t] += rew;
    }
    _rewards = rewardSum;
    return _context->fixedTrajectories;
}