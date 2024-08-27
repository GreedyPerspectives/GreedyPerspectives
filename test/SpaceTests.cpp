/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Space.h"
#include <gtest/gtest.h>

TEST(SpaceTests, StateSpaceIdTest)
{
    unsigned int rows = 10;
    unsigned int cols = 10;
    unsigned int headings = 3;
    unsigned int horizon = 5;
    space::StateSpace ss(rows, cols, headings, horizon);
    space::StateId id = 0;
    for (unsigned int x = 0; x < rows; x++)
    {
        for (unsigned int y = 0; y < cols; y++)
        {
            for (unsigned int r = 0; r < headings; r++)
            {
                for (unsigned int t = 0; t < horizon; t++)
                {
                    EXPECT_EQ(ss.decodeState(id), space::State({x,y,r,t}));
                    EXPECT_EQ(id, ss.encodeState({x,y,r,t}));
                    id++;
                }
                
            }
            
        }
        
    }
}

TEST(SpaceTests, ActionSpaceIdTest)
{
    space::ActionSpace as(sqrt(2), 1);
    for(int i = 0; i < 27; i++)
        EXPECT_EQ(as.actions[i], as.decodeAction(i));
}

// Run the tests
int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}