/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar
 * Email: arauniya@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar.
 ******************************************************************************/

#include "MapGenerator.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <random>

namespace map_generator
{

    Map::Map(float _gridSize, MapSize mapSize, double maxHeight, int obstacleCluster,
             int clusterSize, int seedValue)
      : _type(MapType::RANDOM_GRID)
      , _gridSize(_gridSize)
      , _maxHeight(maxHeight)
      , _mapSize(mapSize)
    {

        setRandomGridMap(obstacleCluster, clusterSize, seedValue);
    }

    MapType Map::getType() const
    {
        return _type;
    }

    float Map::getGridSize() const
    {
        return _gridSize;
    }

    MapSize Map::getMapSize() const
    {
        return _mapSize;
    }

    const HeightMap & Map::getElevationMap() const
    {
        return _elevationMap;
    }

    BinaryOccupancyGrid Map::getObstacleMap(double heightMeters)
    {

        if(_obstacleMapCache.contains(heightMeters))
            return _obstacleMapCache[heightMeters];

        double obsHeight = heightMeters / _gridSize;
        BinaryOccupancyGrid obstacleMap(_mapSize.row, _mapSize.col);
        for(size_t i = 0; i < _mapSize.row; i++)
        {
            for(size_t j = 0; j < _mapSize.col; j++)
            {
                obstacleMap(i, j) = _elevationMap(i, j) >= obsHeight;
            }
        }
        _obstacleMapCache[heightMeters] = obstacleMap;

        return obstacleMap;
    }

    std::vector<Point> generateTightNeighbouringPoints(const Point & point,
                                                       const int totalNeighbours,
                                                       const HeightMap & map)
    {

        std::vector<Point> neighboringPoints;
        int dx[] = {-1, 0, 1, 0}; // Relative x-coordinate changes (left, up, right, down)
        int dy[] = {0, -1, 0, 1}; // Relative y-coordinate changes (left, up, right, down)

        // A point is a neighbour to itself
        neighboringPoints.push_back(point);

        int neighbourCount(0);

        while(neighbourCount < totalNeighbours)
        {

            // Fixing a local center to take the neighbours from
            Point localCenter = neighboringPoints[neighbourCount];

            for(int i = 0; i < 4 && neighbourCount < totalNeighbours; i++)
                for(int j = 0; j < 4 && neighbourCount < totalNeighbours; j++)
                {

                    Point neighbourPoint = {localCenter.x + dx[i], localCenter.y + dy[j]};

                    if(neighbourPoint.x < map.rows() && neighbourPoint.x >= 0
                       && neighbourPoint.y < map.cols() && neighbourPoint.y >= 0)

                        if(map(neighbourPoint.x, neighbourPoint.y) == 0)
                        {
                            neighboringPoints.push_back(neighbourPoint);
                            neighbourCount++;
                        }
                }
        }

        return neighboringPoints;
    }

    void Map::setRandomGridMap(int obstacleCluster, const int & clusterSize,
                               const int & seedValue)
    {

        // Setting the seed for the random generator
        srand(seedValue);

        int rows(_mapSize.row), cols(_mapSize.col);
        _elevationMap.resize(rows, cols);
        _elevationMap.fill(0);

        int obstaclesCount = std::min(obstacleCluster * clusterSize, rows * cols);

        while(obstaclesCount > 0)
        {
            int x = rand() % rows;
            int y = rand() % cols;

            // Generating neighbouring points from the coordinate (x,y)
            Point clusterCenter = {x, y};
            std::vector<Point> neighbouringPoints =
                map_generator::generateTightNeighbouringPoints(
                    clusterCenter, clusterSize, _elevationMap);

            for(auto && neighbourPoint : neighbouringPoints)
            {
                if(_elevationMap(neighbourPoint.x, neighbourPoint.y) == 0)
                {
                    _elevationMap(neighbourPoint.x, neighbourPoint.y) =
                        ((double)rand() / RAND_MAX) * _maxHeight;

                    // Update the obstacle count
                    obstaclesCount--;
                }
            }
        }
    }

    std::vector<float> Map::getMapDataVector() const
    {
        // TODO use map pose in world here
        Eigen::Matrix<float, -1, -1, Eigen::RowMajor> vertexData(_elevationMap.size(), 6);
        for(int i = 0; i < _elevationMap.rows(); i++)
        {
            for(int j = 0; j < _elevationMap.cols(); j++)
            {
                float e = _colorMode == geometry::polygon::ColorMode::GRAYSCALE
                              ? 1 - (_elevationMap(i, j) / _maxHeight)
                              : 0.0f;
                Eigen::Matrix<float, 1, 3> c;
                c.setConstant(e);
                vertexData.row(i * _elevationMap.cols() + j) << i * _gridSize,
                    j * _gridSize, _elevationMap(i, j) * _gridSize, c;
            }
        }
        std::size_t sz = vertexData.innerSize() * vertexData.outerSize();
        std::vector<float> ret(sz);
        float * tmp = vertexData.data();
        std::copy(tmp, tmp + sz, ret.begin());
        return ret;
    }

} // namespace map_generator