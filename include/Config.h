/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_CONFIG_H
#define VISUALGROUPCOVERAGEPLANNER_CONFIG_H

#include "Geometry.h"
#include <array>
#include <cnpy/cnpy.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace config
{
    struct TimeConfig
    {
        int horizon;
        double stepSize;
    };

    struct DiscretizationConfig
    {
        int heading;
        double gridSize;
        double maxTranslationVel;
        double maxRotationVel;
    };

    struct CameraConfig
    {
        int focal;
        int width;
        int height;
        geometry::Pose3D poseOnDrone;
    };

    struct GeometryConfig
    {
        int width;
        int height;
        int length;
    };

    struct ActorConfig
    {
        std::vector<geometry::Pose3D> trajectory;
        GeometryConfig geometryConfig;
        static ActorConfig fromYaml(const YAML::Node & actor_cfg_yaml);
    };

    struct AgentConfig
    {
        geometry::Pose3D startPose; // x,y,z,theta
        std::vector<geometry::Pose3D> trajectory;
        CameraConfig cameraConfig;
        GeometryConfig geometryConfig;
        static AgentConfig fromYaml(const YAML::Node & agent_cfg_yaml);
    };
    struct MapConfig
    {
        geometry::Pose3D pose;
        double gridSize;
        Eigen::MatrixXd heightMap;
    };

    struct WorldConfig
    {
        MapConfig mapConfig;
        TimeConfig time;
        DiscretizationConfig discretizationConfig;
        std::vector<ActorConfig> actorConfigs;
        std::vector<AgentConfig> agentConfigs;

        static WorldConfig fromYaml(const YAML::Node & world_cfg_yaml);
    };

    enum PlannerType
    {
        SEQUENTIAL,
        OPTIMAL,
        MYOPIC,
        FORMATION
    };

    struct PlannerConfig
    {
        PlannerType type;
        int iterations;
        int orientationResolution;
        bool interRobotCollisionConstraints;
        WorldConfig worldCfg;
        static PlannerConfig fromYaml(const YAML::Node & planner_cfg_yaml);
    };

} // namespace config
#endif // VISUALGROUPCOVERAGEPLANNER_CONFIG_H