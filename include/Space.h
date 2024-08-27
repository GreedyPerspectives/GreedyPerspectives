/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_SPACE_H
#define VISUALGROUPCOVERAGEPLANNER_SPACE_H

#include "Geometry.h"
#include <algorithm>
#include <iterator>
#include <limits.h>
#include <optional>

using namespace geometry;

namespace space
{
    typedef size_t StateId;
    typedef size_t ActionId;

    typedef std::array<int, 4> Action;         // delta(x, y, heading, time)
    typedef std::array<unsigned int, 4> State; // x, y, heading, time

    constexpr State terminalState = {UINT_MAX, UINT_MAX, UINT_MAX, UINT_MAX};
    constexpr StateId terminalStateId = -1;
    constexpr Action terminalAction = {INT_MAX, INT_MAX, INT_MAX, INT_MAX};
    constexpr ActionId terminalActionId = -1;

    Pose3D toPose3D(State state, double gridSize, int headings, double height);
    typedef std::vector<std::pair<ActionId, Action>> ActionList;

    // Hasher to allow for constant time lookup
    struct StateHasher
    {
        std::size_t operator()(const State & s) const
        {
            std::size_t h = 0;

            for(auto e : s)
            {
                h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
            }
            return h;
        }
    };
    struct ActionSpace
    {
        // Not great to have 3 representations of the actions but its just used for quick
        // lookups
        ActionList actionList;
        std::vector<Action> actions;
        std::map<Action, ActionId> actionIds;
        int radius;
        int headingCount;
        int maxRotation;
        ActionSpace(double maxTranslationRadius = sqrt(2), int maxRotationDelta = 1);

        // Convert back and forth from ID and action
        ActionId encodeAction(Action action) const
        {
            // TODO check if action time step is not equal to 1
            return actionIds.at(action);
        }
        Action decodeAction(ActionId id) const
        {
            // TODO error if not exist
            return actions[id];
        }
        size_t size() const
        {
            return actions.size();
        };
    };

    struct StateSpace
    {
        int _gridRows;
        int _gridCols;
        int _headings;
        int _horizon;
        StateSpace(int gridRows, int gridCols, int headings, int horizon);

        // Convert back and forth from ID and state
        StateId encodeState(State state) const
        {
            if(state == space::terminalState)
                return space::terminalStateId;
            return state[0] * _gridCols * _headings * _horizon
                   + state[1] * _headings * _horizon + state[2] * _horizon + state[3];
        }
        State decodeState(StateId id) const
        {
            if(id == space::terminalStateId)
                return space::terminalState;
            unsigned int idI = static_cast<unsigned int>(id);
            //clang-format off
            return {
                (idI / (_gridCols * _headings * _horizon)) % _gridRows,
                (idI / (_headings * _horizon)) % _gridCols,
                (idI / _horizon) % _headings,
                (idI % _horizon),
            };
            //clang-format on
        }
        size_t size() const
        {
            return _gridRows * _gridCols * _headings * _horizon;
        };

        State applyAction(State s, Action a) const;
    };

} // namespace space

#endif // VISUALGROUPCOVERAGEPLANNER_SPACE_H