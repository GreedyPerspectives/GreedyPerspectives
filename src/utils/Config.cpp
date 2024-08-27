/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "Config.h"

namespace config
{
    // TODO Add error handling
    ActorConfig ActorConfig::fromYaml(const YAML::Node & actor_cfg_yaml)
    {
        ActorConfig config;
        for(auto & xytheta : actor_cfg_yaml["trajectory"])
        {
            config.trajectory.push_back(geometry::pose3d::fromPositionHeading(
                {xytheta[0].as<double>(), xytheta[1].as<double>(), 0},
                xytheta[2].as<double>()));
        }
        config.geometryConfig.height = actor_cfg_yaml["geometry"]["height"].as<double>();
        config.geometryConfig.length = actor_cfg_yaml["geometry"]["length"].as<double>();
        config.geometryConfig.width = actor_cfg_yaml["geometry"]["width"].as<double>();
        return config;
    }
    AgentConfig AgentConfig::fromYaml(const YAML::Node & agent_cfg_yaml)
    {
        AgentConfig config;
        config.cameraConfig.focal = agent_cfg_yaml["camera"]["intrinsics"][0].as<int>();
        config.cameraConfig.width = agent_cfg_yaml["camera"]["intrinsics"][1].as<int>();
        config.cameraConfig.height = agent_cfg_yaml["camera"]["intrinsics"][2].as<int>();
        config.cameraConfig.poseOnDrone.setIdentity();

        geometry::pose3d::rotateX(config.cameraConfig.poseOnDrone,
                                  agent_cfg_yaml["camera"]["tilt"].as<double>());

        config.geometryConfig.height = agent_cfg_yaml["geometry"]["height"].as<double>();
        config.geometryConfig.length = agent_cfg_yaml["geometry"]["length"].as<double>();
        config.geometryConfig.width = agent_cfg_yaml["geometry"]["width"].as<double>();

        config.startPose.setIdentity();
        config.startPose.translation() << agent_cfg_yaml["startPose"][0].as<double>(),
            agent_cfg_yaml["startPose"][1].as<double>(),
            agent_cfg_yaml["startPose"][2].as<double>();
        geometry::pose3d::rotateZ(config.startPose,
                                  agent_cfg_yaml["startPose"][3].as<double>());

        return config;
    }
    WorldConfig WorldConfig::fromYaml(const YAML::Node & world_cfg_yaml)
    {
        WorldConfig config;

        config.time.horizon = world_cfg_yaml["time"]["horizon"].as<int>();
        config.time.stepSize = world_cfg_yaml["time"]["stepSize"].as<double>();

        config.mapConfig.pose = geometry::Pose3D::Identity();
        // geometry::pose3d::fromXYZRPY(
        //     world_cfg_yaml["map"]["pose"].as<std::array<double, 6>>());
        config.mapConfig.gridSize = world_cfg_yaml["map"]["gridSize"].as<double>();
        auto path = world_cfg_yaml["map"]["file"].as<std::string>();
        auto data = cnpy::npy_load(path);
        config.mapConfig.heightMap = Eigen::Map<Eigen::MatrixXd>(
            data.data<double>(), data.shape[0], data.shape[1]);
        config.mapConfig.heightMap.transposeInPlace();

        for(auto & actorNode : world_cfg_yaml["actors"])
            config.actorConfigs.push_back(ActorConfig::fromYaml(actorNode["actor"]));

        if(world_cfg_yaml["formation"].IsDefined())
        {
            double radius = world_cfg_yaml["formation"]["radius"].as<double>();
            int agentCount = world_cfg_yaml["formation"]["agents_per_actor"].as<int>();
            double angularSpacing =
                agentCount == 2 ? (M_PI / 2) : (M_PI * 2 / agentCount);
            double droneHeight = world_cfg_yaml["formation"]["height"].as<double>();
            spdlog::debug("Angular Spacing {} ", angularSpacing);

            AgentConfig agentConfig;
            agentConfig.startPose = geometry::Pose3D::Identity();
            // TODO load from config
            agentConfig.cameraConfig.focal = 2500;
            agentConfig.cameraConfig.width = 4000;
            agentConfig.cameraConfig.height = 3000;
            agentConfig.cameraConfig.poseOnDrone.setIdentity();
            geometry::pose3d::rotateX(
                agentConfig.cameraConfig.poseOnDrone,
                world_cfg_yaml["formation"]["camera"]["tilt"].as<double>());
            config.agentConfigs =
                std::vector(agentCount * config.actorConfigs.size(), agentConfig);
            std::vector<geometry::Pose3D> agentPoses;
            for(int i = 0; i < agentCount; i++)
            {
                geometry::Pose3D pose;
                pose.setIdentity();
                double ang = i * angularSpacing;
                pose.translate(
                    Eigen::Vector3d{cos(ang) * radius, sin(ang) * radius, droneHeight});
                geometry::pose3d::rotateZ(pose, (M_PI / 2) + ang);
                agentPoses.push_back(pose);
            }

            for(int i = 0; i < config.actorConfigs.size(); i++)
            {
                for(auto & pose : config.actorConfigs[i].trajectory)
                {
                    for(int j = 0; j < agentCount; j++)
                    {
                        config.agentConfigs[j + i * agentCount].trajectory.push_back(
                            pose * agentPoses[j]);
                    }
                }
            }
        }
        else
        {
            config.discretizationConfig.gridSize =
                world_cfg_yaml["state_resolution"]["gridSize"].as<double>();
            config.discretizationConfig.heading =
                world_cfg_yaml["state_resolution"]["heading"].as<int>();
            config.discretizationConfig.maxTranslationVel =
                world_cfg_yaml["action_space"]["max_translation_velocity"].as<double>();
            config.discretizationConfig.maxRotationVel =
                world_cfg_yaml["action_space"]["max_rotation_speed"].as<double>();
            for(auto & agentNode : world_cfg_yaml["agents"])
                config.agentConfigs.push_back(AgentConfig::fromYaml(agentNode["agent"]));
        }
        return config;
    }
    PlannerConfig PlannerConfig::fromYaml(const YAML::Node & yaml)
    {
        YAML::Node planner_cfg_yaml = yaml["planner"];
        PlannerConfig config;
        if(planner_cfg_yaml["method"].as<std::string>() == "sequential")
        {
            config.type = PlannerType::SEQUENTIAL;
            config.iterations =
                planner_cfg_yaml["iterations"].as<int>(); // TODO Positive check
            config.interRobotCollisionConstraints =
                planner_cfg_yaml["inter_robot_collision_constraints"].as<bool>();
        }
        else if(planner_cfg_yaml["method"].as<std::string>() == "formation")
        {
            config.type = PlannerType::FORMATION;
            config.orientationResolution =
                planner_cfg_yaml["orientation_resolution"].as<int>();
        }
        else
        {
            spdlog::error("Invalid Planning Method Specified {}",
                          planner_cfg_yaml["method"].as<std::string>());
            // TODO throw
        }
        config.worldCfg = WorldConfig::fromYaml(planner_cfg_yaml["world"]);
        return config;
    }
} // namespace config