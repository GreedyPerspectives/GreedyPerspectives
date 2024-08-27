/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "Model.h"

DiscreteAgent::DiscreteAgent(const AgentId id, const State start, const StateSpace ss,
                             const ActionSpace as, const size_t horizon,
                             std::shared_ptr<DiscreteContext> context)
  : _id(id)
  , _startState(start)
  , _stateSpace(ss)
  , _actionSpace(as)
  , _horizon(horizon)
  , _context(context)
{}
ActionList DiscreteAgent::availableActions(State state, size_t timeStep,
                                           bool collisionConstraints) const
{
    ActionList actions = {{space::terminalActionId, space::terminalAction}};
    if(state == space::terminalState)
        return actions; // Can't leave terminal state

    // TODO can do some caching here

    // Looping over translations directly since heading can't cause collisions
    for(int i = 0; i < _actionSpace.actions.size(); i += _actionSpace.headingCount)
    {
        space::Action translation = _actionSpace.actions[i];

        // check action within map bounds
        State newState = _stateSpace.applyAction(state, translation);
        if(newState[0] >= _stateSpace._gridRows || newState[0] < 0
           || newState[1] >= _stateSpace._gridCols || newState[1] < 0)
        {
            continue;
        }

        // check map collision
        if(_context->_obstacleMap(newState[0], newState[1]))
        {
            spdlog::debug("Collision Detected at {} {}", newState[0], newState[1]);
            continue;
        }

        // check inter-robot collisions
        if(collisionConstraints
           && _context->_agentCollisionMap[newState[3]].coeff(newState[0], newState[1]))
        {
            continue;
        }

        // Translation is valid so add all heading actions
        for(int h = 0; h < _actionSpace.headingCount; h++)
            actions.push_back(
                {i + h,
                 {translation[0], translation[1], h - _actionSpace.maxRotation, 1}});
    }
    return actions;
}

namespace model
{
    AIToolbox::MDP::SparseModel single_agent::computeMDP(
        const DiscreteAgent & agent, AgentRewardFunction rewardFunction,
        bool collisionConstraints)
    {
        spdlog::debug("Agent [{}], creating MDP...", agent._id);
        // Setup empty T and R matrix
        size_t S = agent._stateSpace.size();
        size_t A = agent._actionSpace.size();
        AIToolbox::SparseMatrix3D transitionMatrix(A, Eigen::SparseMatrix<double>(S, S));
        AIToolbox::SparseMatrix2D rewardMatrix(S, A);
        spdlog::debug("Agent [{}], empty matrices created", agent._id);

        // Initialize start for BFS
        space::State start = agent._startState;
        typedef std::unordered_map<
            space::StateId,
            std::vector<std::pair<space::StateId, space::ActionId>>>
            SASPair;
        SASPair statesToExplore = {{agent._stateSpace.encodeState(start), {}}};
        spdlog::debug("Agent [{}], BFS initalized", agent._id);

        for(int t = 0; t < agent._horizon; t++)
        {
            spdlog::info("Exploring {} states \t time: {}", statesToExplore.size(), t);
            SASPair nextStates;
            for(auto & [stateId, lastStateActions] : statesToExplore)
            {
                // Compute state reward
                space::State state = agent._stateSpace.decodeState(stateId);
                spdlog::debug("Exploring State: {} \t", state);
                if(agent._context->_obstacleMap(state[0], state[1]))
                    spdlog::warn("State in collision");

                // Link to last state action pair
                if(t > 0)
                {
                    for(auto & [lastStateId, actionId] : lastStateActions)
                    {
                        transitionMatrix[actionId].coeffRef(lastStateId, stateId) = 1.0;
                    }
                }

                // TODO for some reason this is causing issues
                // No need to compute actions for terminal state
                // if(t == agent._horizon - 1)
                //     continue;

                // Branch to next set of states
                space::ActionList actions =
                    agent.availableActions(state, t, collisionConstraints);
                for(auto & [actionId, action] : actions)
                {
                    double r = rewardFunction(agent, state, action);
                    spdlog::debug("Computed reward: {}", r);
                    rewardMatrix.coeffRef(stateId, actionId) = r;
                    // Apply action and compute new stateId
                    space::State nextState = agent._stateSpace.applyAction(state, action);
                    if(nextState[3] < 0)
                    {
                        spdlog::error("{} {}", state, action);
                        spdlog::error("Negative State Detected {}", nextState);
                        exit(-1);
                    }
                    space::StateId nextStateId = agent._stateSpace.encodeState(nextState);
                    if(nextStateId == -1)
                        continue; // TODO use something better

                    if(!nextStates.contains(nextStateId))
                        nextStates[nextStateId] = {{stateId, actionId}};
                    else
                        nextStates[nextStateId].emplace_back(stateId, actionId);
                }
            }

            statesToExplore.clear();
            statesToExplore = nextStates;
        }

        return AIToolbox::MDP::SparseModel(AIToolbox::NO_CHECK,
                                           S,
                                           A,
                                           std::move(transitionMatrix),
                                           std::move(rewardMatrix),
                                           1.0);
    }
} // namespace model