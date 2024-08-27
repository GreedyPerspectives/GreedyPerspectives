/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Actor.h"
ActorId Actor::actorCounter = -1;

Actor::Actor(std::vector<Pose3D> trajectory)
  : Entity(trajectory)
  , _geometry(std::make_shared<polygon::RectangularPrism>(_id, 1, 1, 2.5,
                                                          polygon::ColorMode::PICKING))
  , _id(++actorCounter)
{
    spdlog::info("Actor created with id: [{}]", _id);
};