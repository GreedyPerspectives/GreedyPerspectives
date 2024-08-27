/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_AGENT_H
#define VISUALGROUPCOVERAGEPLANNER_AGENT_H

#include "Camera.h"
#include "Config.h"
#include "Entity.h"
#include "Space.h"
#include <ranges>
#include <stdexcept>
#include <utility>
//    TODO: Add unit tests to capture the Agent constructors
using namespace geometry;
using namespace space;

typedef int AgentId;

class Agent : public Entity
{
private:
    static AgentId agentCounter;

public:
    const AgentId _id;

    Pose3D _startPose;
    Camera _camera;

    Agent() = delete; // must specify a startPose for the agent
    Agent(const Agent &) = default;
    Agent(Agent &&) = default;
    Agent(Pose3D startPose, Camera camera = Camera());
    Agent(config::AgentConfig config);
    // Agent(std::vector<Pose3D> & trajectory, Camera camera = Camera());

    // Delegate constructors
    Agent(float x, float y, float z, float heading, Camera camera = Camera());

    inline double droneHeight() const
    {
        return _startPose.translation()[2];
    };
};

#endif // VISUALGROUPCOVERAGEPLANNER_AGENT_H