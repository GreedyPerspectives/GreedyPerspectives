/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#ifndef VISUALGROUPCOVERAGEPLANNER_MODEL_H
#define VISUALGROUPCOVERAGEPLANNER_MODEL_H

#include "Agent.h"
#include "CoverageRenderer.h"
#include "World.h"
#include <AIToolbox/MDP/SparseModel.hpp>
#include <chrono>
#include <functional>

// TODO Not sure where this should live tbh
// Represents all the joint information used to sequentially plan
class DiscreteContext
{
public:
    DiscreteContext(World & world, std::shared_ptr<CoverageRenderer> renderer,
                    map_generator::BinaryOccupancyGrid obstacleMap)
      : _renderer(renderer)
      , _obstacleMap(obstacleMap)
      , _totalCoverage(world.timeInfo().stepCount + 1)
      , _numCollisions(world.timeInfo().stepCount + 1, 0)
      , _agentCollisionMap(
            world.timeInfo().stepCount + 1,
            Eigen::SparseMatrix<bool>(obstacleMap.rows(), obstacleMap.cols()))
    {}
    void setTrajectory(AgentId id, std::vector<State> traj)
    {
        // save trajectory
        fixedTrajectories[id] = traj;

        for(int i = 0; i < traj.size(); i++)
        {
            // Add trajectory to collision map
            _agentCollisionMap[i].coeffRef(traj[i][0], traj[i][1]) = true;

            // Record coverage
            for(auto & [fId, ppa] : _coverageCache[traj[i]].facePPA) {
                _totalCoverage[i][fId] += ppa;
            }
        }
    }

    void unsetTrajectory(AgentId id)
    {
        std::vector<State> traj = fixedTrajectories[id];
        for(int i = 0; i < traj.size(); i++)
        {
            _agentCollisionMap[i].coeffRef(traj[i][0], traj[i][1]) = false;
            for(auto & [fId, ppa] : _coverageCache[traj[i]].facePPA)
                _totalCoverage[i][fId] -= ppa;
        }
        fixedTrajectories.erase(id);
    }

    std::shared_ptr<CoverageRenderer> _renderer;

    // Solution trajectories updated as they get set
    std::map<AgentId, std::vector<State>> fixedTrajectories;

    // Environment + agent start positions
    map_generator::BinaryOccupancyGrid _obstacleMap;

    // Agent state collisions
    std::vector<Eigen::SparseMatrix<bool>> _agentCollisionMap;

    std::vector<int> _numCollisions;

    // set based on solution traj coverage
    std::vector<std::unordered_map<polygon::FaceId, double, polygon::FaceIdHasher>>
        _totalCoverage;

    // State coverage image cache to avoid recomputing coverage image
    std::unordered_map<space::State, CoverageImage, space::StateHasher> _coverageCache;
};

// MDP representation of the continuous agent
class DiscreteAgent // : TODO should extend some generic Partition or concept
{
public:
    const AgentId _id;
    const State _startState;
    const StateSpace _stateSpace;
    const ActionSpace _actionSpace;
    const size_t _horizon;
    std::shared_ptr<DiscreteContext> _context;

    DiscreteAgent(const AgentId id, const State start, const StateSpace ss,
                  const ActionSpace as, const size_t horizon,
                  std::shared_ptr<DiscreteContext> context);
    ActionList availableActions(State state, size_t timeStep,
                                bool collisionConstraints) const;
};

namespace model
{
    namespace single_agent
    {
        // Reward function form
        typedef std::function<double(DiscreteAgent, space::State, space::Action)>
            AgentRewardFunction;

        // Main function used to generate the MDP and explore the state space with BFS
        AIToolbox::MDP::SparseModel computeMDP(const DiscreteAgent & agent,
                                               AgentRewardFunction rewardFunction,
                                               bool collisionConstraints);
    }; // namespace single_agent
} // namespace model
#endif // VISUALGROUPCOVERAGEPLANNER_MODEL_H