/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "CoveragePlanner.h"

Planner::Planner(config::PlannerConfig config)
{
    spdlog::info("Initializing Planner...");
    _world = std::make_unique<World>(config.worldCfg);
    spdlog::debug("World Created");

    _renderer = std::make_shared<CoverageRenderer>(*_world);
    _renderer->createHeadlessWindow();
    _renderer->glInit();
    spdlog::debug("Coverage Renderer Created");

    if(config.type == config::PlannerType::FORMATION)
    {
        spdlog::info("Planner Type: FORMATION");
        _planFound = true;
        std::vector<std::vector<Pose3D>> jointPlan(_world->agents().size());

        // Empty joint trajectories
        // for actor
        // iterate through trajectory
        // rotate agents about actor center and select maximum coverage
        // Add agents to joint plan
        int agentsPerActor = _world->agents().size() / _world->actors().size();
        int actorCount = 0;
        for(const auto & actor : _world->actors())
        {
            for(int t = 0; t < actor._trajectory.size(); t++)
            {
                double maxCoverage = -9999;
                std::map<AgentId, Pose3D> greedyFormation;
                for(int i = 0; i < config.orientationResolution; i++)
                {
                    double coverageSum = 0;
                    double rotation =
                        2 * M_PI * ((float)i / (float)config.orientationResolution);
                    std::map<AgentId, Pose3D> formation;
                    for(AgentId id = agentsPerActor * actorCount; id < agentsPerActor;
                        id++)
                    {
                        // Rotate agent formation about actor pose
                        Pose3D agentPose = actor._trajectory[t].inverse()
                                           * _world->agent(id)._trajectory[t];
                        agentPose = AngleAxisd(rotation, Vector3d::UnitZ()) * agentPose;
                        agentPose = actor._trajectory[t] * agentPose;
                        formation[id] = agentPose;
                        auto img = _renderer->agentCoverageImage(id, t, agentPose);
                        for(auto [fId, ppa] : img.facePPA)
                        {
                            coverageSum += ppa;
                        }
                    }

                    if(coverageSum > maxCoverage)
                    {
                        maxCoverage = coverageSum;
                        greedyFormation = formation;
                    }
                }

                for(auto [id, pose] : greedyFormation)
                {
                    jointPlan[id].push_back(pose);
                    _world->mutableAgent(id)._trajectory[t] = pose;
                }
            }
            actorCount++;
        }

        // Fill with fixed formation paths
        for(int t = 0; t < _world->timeInfo().stepCount; t++)
        {
            std::unordered_map<geometry::polygon::FaceId,
                               double,
                               geometry::polygon::FaceIdHasher>
                cov;
            for(const auto & agent : _world->agents())
            {
                auto img =
                    _renderer->agentCoverageImage(agent._id, t, agent._trajectory[t]);
                for(auto [fId, ppa] : img.facePPA)
                {
                    if(!cov.contains(fId))
                        cov[fId] = 0;
                    cov[fId] += ppa;
                }
            }

            double coverage = 0;
            for(auto [fId, totalCov] : cov)
            {
                coverage += sqrt(totalCov);
            }
            _totalCoverage.push_back(coverage);
        }
    }
    else
    {
        spdlog::info("Planner Type: SEQUENTIAL");
        std::vector<DiscreteAgent> partitions;
        size_t horizon = _world->timeInfo().stepCount;

        // Convert map sizing to realworld sizing
        double realWorldXMax =
            _world->map().getGridSize() * _world->map().getMapSize().row;
        double realWorldYMax =
            _world->map().getGridSize() * _world->map().getMapSize().col;
        spdlog::debug("Real World Map Dims(meters): width: {} length: {}",
                      realWorldXMax,
                      realWorldYMax);
        StateSpace ss((int)realWorldXMax / _world->disConfig.gridSize,
                      (int)realWorldYMax / _world->disConfig.gridSize,
                      _world->disConfig.heading,
                      horizon);
        spdlog::debug("State Space Shape: {} {} {} {}",
                      ss._gridRows,
                      ss._gridCols,
                      ss._headings,
                      ss._horizon);
        spdlog::debug("State Space Size: {}", ss.size());

        ActionSpace as; // TODO compute from speeds and timing
        spdlog::debug("Action Space Size: {}", as.size());

        // Get collision region for agents at the planning height
        // TODO set per drone specific height and don't assume same height
        auto mapSpaceColMap =
            _world->mutableMap().getObstacleMap(_world->agent(0).droneHeight());

        // Convert realworld collision map to statespace collision map
        map_generator::BinaryOccupancyGrid collisionMap(ss._gridRows, ss._gridCols);
        for(size_t i = 0; i < ss._gridRows; i++)
        {
            for(size_t j = 0; j < ss._gridCols; j++)
            {
                double realWorldX = i * _world->disConfig.gridSize;
                double realWorldY = j * _world->disConfig.gridSize;
                collisionMap(i, j) =
                    mapSpaceColMap((int)(realWorldX / _world->map().getGridSize()),
                                   (int)(realWorldY / _world->map().getGridSize()));
            }
        }

        _context = std::make_shared<DiscreteContext>(*_world, _renderer, collisionMap);
        spdlog::debug("Discretized Context Created");

        for(auto & agent : _world->agents())
        {
            spdlog::debug("Discrete Agent {}", agent._id);
            spdlog::debug("{}", agent._startPose.translation()[0]);

            // Find discrete start state based on realworld position
            unsigned int x =
                agent._startPose.translation()[0] / _world->disConfig.gridSize;
            unsigned int y =
                agent._startPose.translation()[1] / _world->disConfig.gridSize;
            unsigned int heading =
                ((int)(agent._startPose.rotation().eulerAngles(0, 1, 2)[2]
                       / (M_PI * 2.0 / (float)_world->disConfig.heading))
                 + _world->disConfig.heading)
                % _world->disConfig.heading;
            State start = {x, y, heading, 0};
            spdlog::debug("Start State: {}", start);
            if(config.interRobotCollisionConstraints)
                _context->_obstacleMap(x, y) =
                    true; // Set each agent's start position as an
                          // obstacle to prevent future collisions
            partitions.push_back(
                DiscreteAgent(agent._id, start, ss, as, horizon, _context));
        }

        _solver =
            std::make_unique<SubmodularSolver>(partitions,
                                               _context,
                                               &SingleAgentRewardFunction,
                                               config.iterations,
                                               config.interRobotCollisionConstraints);
        spdlog::debug("Solver Created");
    }
}

std::vector<std::vector<Pose3D>> Planner::plan()
{
    if(_planFound)
    {
        spdlog::info("Plan Already Found");
        return _plan;
    }

    spdlog::info("Generating Plan...");
    std::map<AgentId, std::vector<State>> discreteSolution = _solver->solveSequential();
    spdlog::info("Sequential Solution Created");
    std::vector<std::vector<Pose3D>> jointPlan(discreteSolution.size());

    // Convert discrete solution to realworld trajectory
    for(auto & [id, discreteTrajectory] : discreteSolution)
    {
        std::vector<Pose3D> trajectory;
        double droneHeight = _world->agent(id).droneHeight();
        for(auto & state : discreteTrajectory)
        {
            trajectory.push_back(space::toPose3D(state,
                                                 _world->disConfig.gridSize,
                                                 _world->disConfig.heading,
                                                 droneHeight));
        }
        _world->setAgentTrajectory(id, trajectory);
        jointPlan[id] = trajectory;
    }
    int num_robots = _world->agents().size();
    for (int t = 0; t < discreteSolution[0].size(); t++) {
        int cols = 0;
        for (int a =0; a< num_robots-1; a++){
            for (int b=a+1; b< num_robots;b++) {
                if ((discreteSolution[a][t][0] == discreteSolution[b][t][0]) && (discreteSolution[a][t][1] == discreteSolution[b][t][1])){
                    spdlog::info("Colision Found: robot {}, robot {} at time {}, {},{}", discreteSolution[a][t], discreteSolution[b][t], t, a, b);
                    cols++;
                }
            }
        }
        
        spdlog::info("{}, {}", t, cols);
    }

    // Record total coverage measured through the joint trajectory
    int t = 0;
    for(auto & coverage : _context->_totalCoverage)
    {
        double sum = 0;
        for(auto & [fid, ppa] : coverage)
        {
            sum += sqrt(ppa);
        }
        _totalCoverage.push_back(sum);
        spdlog::info("{}, {}", t++, sum);
    }

    // Save joint plan
    _plan = jointPlan;
    _planFound = true;
    return jointPlan;
}

void Planner::visualize()
{
    _renderer->destroyHeadlessWindow();
    Visualizer vs = Visualizer(*_world);
    vs.initializeRenderers();
    vs.displayLoop();
    vs.cleanupRenderers();
}

double Planner::SingleAgentRewardFunction(const DiscreteAgent & agent, space::State state,
                                          space::Action action)
{
    if(state == space::terminalState)
        return -99999; // Large negative reward for terminating
    // Setup
    double reward = 0;
    std::shared_ptr<DiscreteContext> context = agent._context;
    int time = state[3];

    // If state has been observed from previously we can pull data from cache, if not we
    // query the renderer
    if(!context->_coverageCache.contains(state))
    {
        // TODO geometry check
        context->_coverageCache[state] =
            context->_renderer->agentCoverageImage(agent._id, state);
    }

    auto img = context->_coverageCache[state];

    spdlog::trace("Computing coverage reward, agent: [{}], state {}", agent._id, state);
    spdlog::trace("WxH {}x{}. Rescale {}. Faces {}",
                  img.width,
                  img.height,
                  img.rescale,
                  img.facePPA.size());

    // Map to track faces not seen previously
    std::unordered_map<FaceId, double, FaceIdHasher> unseenFaces = img.facePPA;

    spdlog::trace("Computing coverage rewards");
    for(auto & [fId, priorPPA] : context->_totalCoverage[time])
    {
        double ppa = 0.0;
        if(img.facePPA.contains(fId))
        {
            unseenFaces.erase(fId); // seen face is removed from list
            ppa = img.facePPA[fId];
        }

        reward += sqrt(ppa + priorPPA);
        spdlog::trace("Current Reward {}", reward);
    }

    spdlog::trace("Adding reward for new faces");
    // add reward for faces not previously observed
    for(auto & [fId, ppa] : unseenFaces)
    {
        reward += sqrt(ppa);
    }

    spdlog::trace("Adding reward for movement");
    // Staying still is rewarded
    if((action[0] + action[1]) == 0)
        reward += 100;

    // No rotation is rewarded
    if(action[3] == 0)
        reward += 200;

    return reward;
}