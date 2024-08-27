/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "World.h"

#include "Logging.h"

World::World(config::WorldConfig worldConfig)
  : _timeInfo({worldConfig.time.stepSize, worldConfig.time.horizon})
  , _map(worldConfig.mapConfig.heightMap, worldConfig.mapConfig.gridSize,
         worldConfig.mapConfig.pose)
{
    disConfig = worldConfig.discretizationConfig;

    // TODO read geometry from config
    for(auto & agentConfig : worldConfig.agentConfigs)
        _agents.emplace_back(agentConfig);
    for(auto & actorConfig : worldConfig.actorConfigs)
        _actors.emplace_back(actorConfig.trajectory);
}

World::World(map_generator::Map map, TimeInfo timeInfo, std::vector<Actor> && actors,
             std::vector<Agent> && agents)
  : _map(map)
  , _timeInfo(timeInfo)
  , _actors(actors)
  , _agents(agents)
{
    if(_actors.size() < 1 || _agents.size() < 1)
    {
        spdlog::error(
            "Attempting to create world with {} actors and {} agents. Must have greater "
            "than 1 for both.",
            _actors.size(),
            _agents.size());
    }
    // TODO handle mismatch agent actor trajectories with the timelength

    spdlog::info(
        "Creating world with {} actors and {} agents.", _actors.size(), _agents.size());
}

void World::setAgentTrajectory(const AgentId id, std::vector<Pose3D> trajectory)
{
    // todo error handling
    // todo update conflict list
    _agents[id]._trajectory = trajectory;
    _agents[id]._trajectoryFixed = true;
}

// World World::MakeRandomWorld(const map_generator::Map & map, const int & actorCount,
//                              const int & agentCount, const int seedValue)
// {

//     // Setting SEED value for the random generating values
//     srand(seedValue);

//     std::vector<Actor> actors;
//     std::vector<Agent> agents;

//     int safetyDistance =
//         map.getGridSize() * 1; // TODO Adjust this based on the drone dimension

//     // Get a coordinate that lies on the obstacle-free space
//     // Set the start position for an agent
//     for(int agentID = 0; agentID < agentCount; agentID++)
//     {
//         spdlog::info("Creating a start position for agent with agentID({})", agentID);

//         bool validCoordinate = false;

//         int x, y;

//         while(!validCoordinate)
//         {
//             x = rand() % map.getMapSize().row;
//             y = rand() % map.getMapSize().col;

//             // Check if the coordinate doesn't lie on the ObstacleMap
//             if(map.getObstacleMap()(x, y) == 0)
//                 validCoordinate = true;

//             // Check if the coordinate is the location of an already assigned agent
//             // location
//             for(int currentAgentID = 0; currentAgentID < agentID && validCoordinate;
//                 currentAgentID++)
//             {
//                 Point3D XYZofCurrentAgent =
//                     agents[currentAgentID]._startPose.translation();

//                 if(XYZofCurrentAgent(0) == x && XYZofCurrentAgent[1] == y
//                    && XYZofCurrentAgent[2] == map.getObstacleMapHeight())
//                     validCoordinate = false;
//             }
//         }
//         Pose3D pose = Pose3D::Identity();
//         pose.translate(Vector3d(x, y, map.getObstacleMapHeight()));
//         Agent newAgent(pose);

//         agents.push_back(newAgent);

//         spdlog::info("Agent({}) set to start pos as ({}, {}, {})",
//                      agentID,
//                      x,
//                      y,
//                      map.getObstacleMapHeight());
//     }

//     // Set the Trajectory for an Actor
//     for(int actorID = 0; actorID < actorCount; actorID++)
//     {
//         spdlog::info("Creating a start position for actor with actorID({})", actorID);

//         bool validCoordinate = false;

//         int x, y;

//         while(!validCoordinate)
//         {
//             x = rand() % map.getMapSize().row;
//             y = rand() % map.getMapSize().col;

//             // Check if the coordinate doesn't lie on the ElevationMap
//             if(map.getElevationMap()(x, y) == 0)
//             {
//                 validCoordinate = true;
//             }

//             // Check if the coordinate is the location of an already assigned agent
//             // location
//             for(int currentactorID = 0; currentactorID < actorID && validCoordinate;
//                 currentactorID++)
//             {
//                 Point3D XYZofCurrentActor =
//                     actors[currentactorID].getPosition(0).translation();

//                 if(XYZofCurrentActor[0] == x && XYZofCurrentActor[1] == y
//                    && XYZofCurrentActor[2] == map.getObstacleMapHeight())
//                 {

//                     spdlog::trace("Coordinate({},{}) has vertex collision with
//                     actor({})",
//                                   x,
//                                   y,
//                                   currentactorID);
//                     validCoordinate = false;
//                 }
//             }
//         }

//         int dx[] = {-1, 0, 1, 0}; // Relative x-coordinate changes (left, up, right,
//         down) int dy[] = {0, -1, 0, 1}; // Relative y-coordinate changes (left, up,
//         right, down)

//         int pathLengthOfActor = rand() % 15;
//         spdlog::trace(
//             "Random path length set for the actor({}) as {}", actorID,
//             pathLengthOfActor);

//         Pose3D pose = Pose3D::Identity();
//         pose.translate(Vector3d(x, y, 0));
//         std::vector<Pose3D> trajectory = {pose};

//         spdlog::trace(
//             "New location for actor({}) at ts({}) is ({},{}, 0) is valid in the "
//             "elevation map",
//             actorID,
//             0,
//             x,
//             y);

//         // Creating new locations till the path length for the actor is met
//         for(int currPathIndex = 0; currPathIndex < pathLengthOfActor - 1;
//         currPathIndex++)
//         {
//             spdlog::trace(
//                 "Finding a valid next location for timstep({}) for the actor({})",
//                 currPathIndex,
//                 actorID);

//             int nextX(x), nextY(y);

//             bool newValidLocation = false;

//             while(!newValidLocation)
//             {
//                 int newDx = rand() % 4;
//                 int newDy = rand() % 4;

//                 int newX = nextX + dx[newDx];
//                 int newY = nextY + dy[newDy];

//                 spdlog::trace("Checking if ({}, {}) is valid.", newX, newY);

//                 if(newX < map.getElevationMap().rows() && newX >= 0
//                    && newY < map.getElevationMap().cols() && newY >= 0
//                    && map.getElevationMap()(newX, newY) == 0)
//                 {

//                     spdlog::trace(
//                         "({}, {}) is obstacle free and inside the map", newX, newY);

//                     newValidLocation = true;
//                 }

//                 // Check if there is no actor-actor collision at this timestep
//                 for(int currentactorID = 0; currentactorID < actorID; currentactorID++)
//                 {

//                     // Get the evaluation timestep as not all the actors have same path
//                     // length
//                     int evalTimestep = std::min(
//                         static_cast<int>(actors[currentactorID]._trajectory.size()),
//                         currPathIndex);

//                     spdlog::trace(
//                         "Checking if actor({}) collides with actor({}) at
//                         timestep({})", actorID, currentactorID, evalTimestep);

//                     Point3D XYZofCurrentActorAtT =
//                         actors[currentactorID].getPosition(evalTimestep).translation();

//                     // Actor-actor vertex collision check
//                     if(XYZofCurrentActorAtT[0] == newX && XYZofCurrentActorAtT[1] ==
//                     newY)
//                     {
//                         spdlog::trace("Collision Seen!");
//                         newValidLocation = false;
//                     }
//                 }

//                 if(newValidLocation)
//                 {
//                     nextX = newX;
//                     nextY = newY;
//                 }
//             }

//             Pose3D newPose = Pose3D::Identity();
//             newPose.translate(Vector3d{(double)nextX, (double)nextY, 0});
//             trajectory.push_back(newPose);
//             spdlog::trace(
//                 "New location for actor({}) at ts({}) is ({},{}, 0) is valid in the "
//                 "elevation map",
//                 actorID,
//                 currPathIndex + 1,
//                 nextX,
//                 nextX);
//         }

//         Actor newActor(trajectory);
//         actors.push_back(newActor);
//     }

//     TimeInfo t = {0.1, (int)actors[0]._trajectory.size()};
//     return World(map, t, std::move(actors), std::move(agents));
// }