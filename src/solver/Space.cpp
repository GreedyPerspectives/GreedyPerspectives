/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "Space.h"

namespace space
{

    State StateSpace::applyAction(State s, Action a) const
    {
        if(s == space::terminalState || a == space::terminalAction)
            return space::terminalState;
        int a_h = (a[2] + _headings) % _headings; // wrap around state heading
        return {s[0] + a[0], s[1] + a[1], (s[2] + a_h) % _headings, s[3] + a[3]};
    }

    Pose3D toPose3D(State state, double gridSize, int headings, double height)
    {
        // Set real world pose from state.
        if(state == space::terminalState)
        {
            spdlog::warn("Terminal state attempted to be translated to Pose");
            return Pose3D::Identity();
        }
        // TODO remove 0.5 addition
        return geometry::pose3d::fromPositionHeading(
            {(state[0] + 0.5) * gridSize, (state[1] + 0.5) * gridSize, height},
            M_PI * 2 * (float)state[2] / (float)headings);
    };

    ActionSpace::ActionSpace(double maxTranslationRadius, int maxRotationDelta)
    {
        radius = static_cast<int>(maxTranslationRadius);
        headingCount = 2 * maxTranslationRadius + 1;
        maxRotation = maxRotationDelta;
        // TODO check if either number is negative
        ActionId id = 0;

        // Step through actionable space and add all elements
        int radSq = maxTranslationRadius * maxTranslationRadius;
        for(int x = -maxTranslationRadius; x <= maxTranslationRadius; x++)
        {
            for(int y = -maxTranslationRadius; y <= maxTranslationRadius; y++)
            {
                if(x * x + y * y <= radSq)
                {
                    for(int r = -maxRotationDelta; r <= maxRotationDelta; r++)
                    {
                        actions.push_back({x, y, r, 1});
                        actionList.push_back({id, actions.back()});
                        actionIds[actions.back()] = id++;
                    }
                }
            }
        }
        actions.push_back(space::terminalAction);
        actionList.push_back({space::terminalActionId, actions.back()});
        actionIds[actions.back()] = space::terminalActionId;
    }

    StateSpace::StateSpace(int gridRows, int gridCols, int headings, int horizon)
      : _gridRows(gridRows)
      , _gridCols(gridCols)
      , _headings(headings)
      , _horizon(horizon)
    {}
} // namespace space