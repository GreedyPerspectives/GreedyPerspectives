/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Entity.h"

Entity::Entity(){};

Entity::Entity(std::vector<Pose3D> trajectory)
  : _trajectory(trajectory)
  , _trajectoryFixed(true){};
Pose3D Entity::getPosition(int timeIndex) const
{
    if(!_trajectoryFixed)
    {
        spdlog::error(
            "Tried to get position for entity at time {}. Trajectory not fixed.",
            timeIndex);
        return Pose3D::Identity();
    }
    else if(timeIndex >= _trajectory.size())
    {
        spdlog::error("Tried to get position for entity at time {}. Out of bounds.",
                      timeIndex);
        return Pose3D::Identity();
    }

    return _trajectory[timeIndex];
};