/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_COVERAGEPLANNER_H
#define VISUALGROUPCOVERAGEPLANNER_COVERAGEPLANNER_H

#include "Config.h"
#include "CoverageRenderer.h"
#include "Logging.h"
#include "Solver.h"
#include "Visualizer.h"
#include "World.h"

class Planner
{
public:
    Planner(config::PlannerConfig config);
    Planner(std::string yamlConfigPath)
      : Planner(config::PlannerConfig::fromYaml(YAML::LoadFile(yamlConfigPath))){};

    // Returns a trajectory for each agent in world
    std::vector<std::vector<Pose3D>> plan();

    void visualize();

    inline std::vector<double> rewardOverTime()
    {
        return _solver->_rewards;
    }
    inline std::vector<double> coverageOverTime()
    {
        return _totalCoverage;
    };
    inline double averageCoverage()
    {
        double sum = 0;
        for(auto ppa : _totalCoverage)
            sum += ppa;
        return sum / _world->timeInfo().stepCount;
    };
    inline void saveImage(std::string filename)
    {
        auto bev = BEVRenderer(*_world, 800, 800);
        bev.saveImage(0, filename + "_s.png");
        bev.saveImage(_world->timeInfo().stepCount - 1, filename + ".png");
    }

private:
    // TODO other planning parameters here
    static double SingleAgentRewardFunction(const DiscreteAgent & agent,
                                            space::State state, space::Action action);
    std::unique_ptr<World> _world;
    std::shared_ptr<CoverageRenderer> _renderer;
    std::shared_ptr<DiscreteContext> _context;
    std::unique_ptr<SubmodularSolver> _solver;
    bool _planFound = false;
    std::vector<std::vector<Pose3D>> _plan;
    std::vector<double> _totalCoverage;
};

#endif // VISUALGROUPCOVERAGEPLANNER_COVERAGEPLANNER_H

// FIXME Create a Coverage Namespace
