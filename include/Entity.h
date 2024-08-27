/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_ENTITY_H
#define VISUALGROUPCOVERAGEPLANNER_ENTITY_H

#include "Geometry.h"
#include "Logging.h"
using namespace geometry;

class Entity
{
public:
    std::vector<Pose3D> _trajectory;
    bool _trajectoryFixed = false;

    Entity();

    Entity(std::vector<Pose3D> trajectory);

    Pose3D getPosition(int timeIndex) const;
};

#endif // VISUALGROUPCOVERAGEPLANNER_ENTITY_H
