/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar
 * Email: arauniya@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar.
 ******************************************************************************/

#ifndef MAP_H
#define MAP_H

#include "Geometry.h"
#include <Eigen/Core>
#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace map_generator
{
    typedef Eigen::Matrix<bool, -1, -1> BinaryOccupancyGrid;
    typedef Eigen::MatrixXd HeightMap;
    // Enum to define the types of Map
    enum class MapType
    {
        RANDOM_GRID,  // Randomly generated grid map
        USER_DEFINED, // User-defined map
        OCTOMAP       // Octomap-based map
    };

    // Struct representing a point in the grid space
    struct Point
    {
        int x;
        int y;
    };

    // Struct representing the size of the map
    struct MapSize
    {
        int row;
        int col;
    };

    // Class representing the Map
    class Map
    {
    public:
        // Constructors
        Map(float gridSize, MapSize mapSize, double maxHeight, int obstacleCluster,
            int clusterSize, int seedValue); // For MapType::RANDOM_GRID

        Map(Eigen::MatrixXd heightMap, double gridSize,
            geometry::Pose3D pose = geometry::Pose3D::Identity())
          : _type(MapType::USER_DEFINED)
          , _gridSize(gridSize)
          , _maxHeight(heightMap.maxCoeff())
          , _elevationMap(heightMap)
          , _mapSize({(int)heightMap.rows(), (int)heightMap.cols()})
          , _pose(pose)
        {}

        // Function to get the type of the Map
        [[nodiscard]] MapType getType() const;

        // Function to get the grid size
        [[nodiscard]] float getGridSize() const;

        // Function to get the Map size
        [[nodiscard]] MapSize getMapSize() const;

        // Function to get the Eigen matrix representing the Elevation Map
        [[nodiscard]] const HeightMap & getElevationMap() const;

        // Function to get the Eigen matrix representing the Obstacle Map
        [[nodiscard]] BinaryOccupancyGrid getObstacleMap(double heightMeters);

        [[nodiscard]] std::vector<float> getMapDataVector() const;

        void setColorMode(geometry::polygon::ColorMode mode)
        {
            _colorMode = mode;
        };
        double _maxHeight;

    private:
        geometry::Pose3D _pose;
        MapType _type;    // Type of the Map
        float _gridSize;  // Grid size for the Map in meters
        MapSize _mapSize; // Size of the Map

        HeightMap _elevationMap; // Eigen matrix representing the Elevation Map
        std::map<double, BinaryOccupancyGrid>
            _obstacleMapCache; // Eigen matrix representing the Obstacle Map at a
                               // certain height
        geometry::polygon::ColorMode _colorMode = geometry::polygon::ColorMode::BLACK;
        // Private functions for generating the random grid Map
        void setRandomGridMap(int obstacleCluster, const int & clusterSize,
                              const int & seedValue);

        // Private function for generating 'n' neighboring points from a point in the grid
        // space
        std::vector<Point> generateTightNeighbouringPoints(const Point & point,
                                                           const int totalNeighbours,
                                                           const HeightMap & map);
    };

} // namespace map_generator

#endif // MAP_H
