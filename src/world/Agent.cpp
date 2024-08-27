/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Agent.h"

AgentId Agent::agentCounter = -1;

// TODO check if start pose is inside map bounds
Agent::Agent(Pose3D startPose, Camera camera)
  : _startPose(startPose)
  , _camera(camera)
  , _id(++agentCounter)
{
    spdlog::info("Agent created with id: [{}]", _id);
};

Agent::Agent(float x, float y, float z, float heading, Camera camera)
  : Agent(pose3d::fromPositionHeading({x, y, z}, heading), camera){};

Agent::Agent(config::AgentConfig config)
  : Agent(config.startPose,
          Camera(config.cameraConfig.focal, config.cameraConfig.width,
                 config.cameraConfig.height, config.cameraConfig.poseOnDrone))
{
    // TODO move to init list
    if(config.trajectory.size() > 0)
    {

        _trajectory = config.trajectory;
        _trajectoryFixed = true;
    }
}
// Agent::Agent(std::vector<Pose3D> & trajectory, Camera camera)
//   : Entity(trajectory)
//   , _startPose(trajectory.front())
//   , _camera(camera)
//   , _id(++agentCounter)
// {
//     if(trajectory.empty())
//     {
//         spdlog::error("Failed to create Agent with empty trajectory");
//         throw std::invalid_argument("");
//     }
// }