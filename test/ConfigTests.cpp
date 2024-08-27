/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Config.h"
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

TEST(ConfigTests, YamlLoadTest)
{
    // TODO
    // std::string path = "../example.yaml";
    // YAML::Node node = YAML::LoadFile(path);
    // auto config = config::PlannerConfig::fromYaml(node);
    // spdlog::info("{}", config.worldCfg.actorConfigs.size());
    // spdlog::info("{}", config.worldCfg.agentConfigs.size());
}

// Run the tests
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}