/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar
 * Email: arauniya@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar.
 ******************************************************************************/

//
// Created by Adi on 6/8/2023.
//

#include <gtest/gtest.h>
#include "MapGenerator.h"

namespace map_generator {

    // Test fixture class for MapGenerator tests
    class MapGeneratorTest : public ::testing::Test {
    protected:
        // Set up common configurations for the tests
        // Create a MapGenerator object with a random grid HeightMap
        float gridSize = 1.0; // Meters
        int rows = 20, cols = 30;
        double maxHeight = 25;
        MapSize mapSize = {rows, cols};
        int obstacleCluster = 6, clusterSize = 10;
        int seedValue = 10;

        Map *randomMapPtr;

        void SetUp() override {
            // Create a new Map object with the specified parameters
            randomMapPtr = new Map(gridSize, mapSize, maxHeight, obstacleCluster, clusterSize, seedValue);
        }

        void TearDown() override {
            // Clean up after the tests
            delete randomMapPtr;
        }
    };

    // // Test case to check the assignment of map type and grid size
    // TEST_F(MapGeneratorTest, AssignmentTest) {
    //     // Uncomment to view the generated Map
    //     for (int i = 0; i < int(rowsInMeters / gridSize); i++) {
    //         for (int j = 0; j < int(colsInMeters / gridSize); j++) {
    //             std::cout << randomMapPtr->getElevationMap()(i, j) << " ";
    //         }
    //         std::cout << std::endl;
    //     }

    //     // Assignment Checks
    //     EXPECT_EQ(randomMapPtr->getType(), MapType::RANDOM_GRID);
    //     EXPECT_EQ(randomMapPtr->getGridSize(), gridSize);
    // }

    // // Test case to check the generation of obstacles
    // TEST_F(MapGeneratorTest, ObstacleGenerationTest) {
    //     int totalObstacles = obstacleCluster * clusterSize;
    //     int obstaclesCreated = 0;

    //     // Check for the total obstacles created based on obstacle cluster and clusterSize
    //     for (int i = 0; i < randomMapPtr->getElevationMap().rows(); ++i) {
    //         for (int j = 0; j < randomMapPtr->getElevationMap().cols(); ++j) {
    //             if (randomMapPtr->getElevationMap()(i, j) > 0) {
    //                 obstaclesCreated++;
    //                 if (randomMapPtr->getElevationMap()(i, j) > obstacleMapHeight)
    //                     EXPECT_EQ(randomMapPtr->getObstacleMap()(i, j), 1);
    //             }
    //             EXPECT_LT(randomMapPtr->getElevationMap()(i, j), mapSize.height);
    //         }
    //     }

    //     // EXPECT_LT(std::abs(obstaclesCreated-totalObstacles), 5);
    // }

}  // namespace map_generator

// Run the tests
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
