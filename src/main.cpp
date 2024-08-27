/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Config.h"
#include "CoveragePlanner.h"
#include "CoverageRenderer.h"
#include "Logging.h"
#include "Visualizer.h"
#include "World.h"
#include <iostream>

int main(int argc, char * argv[])
{
    logging::init();
    bool viz = false;
    // Take config path as argument but use example as a default
    std::string path = "example.yaml";
    if(argc == 2)
        path = argv[1];
    if(argc == 3)
    {
        path = argv[1];
        viz = true;
    }

    spdlog::info("Loading Planner Config: {}", path);

    YAML::Node node = YAML::LoadFile(path);
    auto config = config::PlannerConfig::fromYaml(node);

    Planner planner(config);
    auto plan = planner.plan();

    // Print out the coverage over time for the generated plan
    int t = 0;
    for(auto ppa : planner.coverageOverTime())
        spdlog::info("Time, PPA: {}, {}", t++, ppa);
    spdlog::info("Average Coverage: {}", planner.averageCoverage());

    auto time = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time),
                        "%F_%T"); // ISO 8601 without timezone information.
    auto s = ss.str();
    std::replace(s.begin(), s.end(), ':', '-');

    std::string pre = "";
    if(node["planner"]["method"].as<std::string>() == "formation")
    {
        pre = "form_";
    }
    else
    {

        pre =
            "seq_" + node["planner"]["iterations"].as<std::string>() + "_"
            + (node["planner"]["inter_robot_collision_constraints"].as<bool>() ? "col"
                                                                               : "nocol")
            + "_";
    }
    std::size_t pos = path.find_last_of('/');

    if(pos != std::string::npos)
        pre = path.substr(0, pos) + "/" + pre;

    std::ofstream file(pre + s + ".csv");
    file << "#avg_view_reward: " + std::to_string(planner.averageCoverage()) << "\n";
    file << "view_reward,planner_reward\n";
    for(int t = 0; t < planner.coverageOverTime().size(); t++)
        file << planner.coverageOverTime()[t] << ","
             << (node["planner"]["method"].as<std::string>() == "formation"
                     ? 0.0
                     : planner.rewardOverTime()[t])
             << "\n";
    file.close();

    planner.saveImage(pre + s);

    // Display visualization window
    if(viz)
        planner.visualize();

    return 0;
}
