/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_ACTOR_H
#define VISUALGROUPCOVERAGEPLANNER_ACTOR_H

#include "Entity.h"
#include <memory>

using namespace geometry;

typedef int ActorId;
class Actor : public Entity
{
private:
    static ActorId actorCounter;

public:
    const ActorId _id;
    std::shared_ptr<polygon::Polygon3D> _geometry;
    //    TODO: Add unit tests to capture the Actor constructors
    Actor() = delete; // must specify a trajectory for the actor
    Actor(const Actor &) = default;
    Actor(Actor &&) = default;
    Actor(std::vector<Pose3D> trajectory);

    void setColorMode(geometry::polygon::ColorMode mode)
    {
        _geometry = std::make_shared<polygon::RectangularPrism>(_id, 1, 1, 2.5, mode);
    }
};

#endif // VISUALGROUPCOVERAGEPLANNER_ACTOR_H
