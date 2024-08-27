/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#ifndef VISUALGROUPCOVERAGEPLANNER_WORLD_H
#define VISUALGROUPCOVERAGEPLANNER_WORLD_H

#include "Actor.h"
#include "Agent.h"
#include "Config.h"
#include "Space.h"
#include <MapGenerator.h>
#include <algorithm>
#include <random>
#include <vector>

// Time is handled by time index where the real-world time is
// timeStep*timeIndex

// FIXME  Add unittest to capture world creation and

struct TimeInfo
{
    double timeStepLength;
    int stepCount;
};

struct CoverageImage
{
    std::unordered_map<polygon::FaceId, double, polygon::FaceIdHasher> facePPA;
    int width;
    int height;
    int rescale;
};
class World
{
private:
    const TimeInfo _timeInfo;

    map_generator::Map _map;
    std::vector<Actor> _actors;
    std::vector<Agent> _agents;

public:
    config::DiscretizationConfig disConfig;
    World(config::WorldConfig worldConfig);

    explicit World(map_generator::Map map, TimeInfo timeInfo,
                   std::vector<Actor> && actors = {}, std::vector<Agent> && agents = {});

    // All the get functions to get the private members //
    inline const std::vector<Agent> & agents() const
    {
        return _agents;
    }
    inline const Agent & agent(AgentId id) const
    {
        // TODO check if exists
        return _agents[id];
    }

    inline Agent & mutableAgent(AgentId id)
    {
        // TODO check if exists
        return _agents[id];
    }

    inline const std::vector<Actor> & actors() const
    {
        return _actors;
    }

    inline std::vector<Actor> & mutableActors()
    {
        return _actors;
    }

    inline const map_generator::Map & map() const
    {
        return _map;
    }

    inline map_generator::Map & mutableMap()
    {
        return _map;
    }

    inline const TimeInfo & timeInfo() const
    {
        return _timeInfo;
    };

    // Interface functions //
    void setAgentTrajectory(const AgentId id, std::vector<Pose3D> trajectory);

    // TODO these functions below should be moved to a world helper static namespace //
    // Function to create a random valid world
    // where agents are assigned a valid start location and actor a valid trajectory
    // static World MakeRandomWorld(const map_generator::Map & map, const int &
    // actorCount,
    //                              const int & agentCount, const int seedValue);

    // Function to assign actor with actorID to a user-defined trajectory
    void setRandomActorTrajectoryFor(const int & actorID);

    // Function to assign agent with agentID to a user-defined start position
    void setRandomAgentStartPosFor(const int & agentID);
};

#endif // VISUALGROUPCOVERAGEPLANNER_WORLD_H