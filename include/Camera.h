/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_CAMERA_H
#define VISUALGROUPCOVERAGEPLANNER_CAMERA_H

#include "Geometry.h"
#include "Logging.h"
#include <array>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <memory>
#include <vector>

// TODO: Make the Tilt variable such that we can perform some optimization based path
// after view planning
#define DEFAULT_CAMERA_TILT 1.3   // rad
#define DEFAULT_CAMERA_OFFSET 0.5 // meters
#define NEAR_PLANE 0.1f
#define FAR_PLANE 1000.0f

using namespace geometry;

class Camera
{
public:
    const int _id;
    static int cameraCount;
    // Extrinsic in agent frame
    Pose3D _agentToCameraTransform;
    // Intrinsics
    Matrix3d _cameraIntrinsicsMatrix;
    const double _focalLength; // Pixels
    const double _imageWidth;  // Pixels
    const double _imageHeight; // Pixels
    const double _fovY;        // Degrees
    const double _aspectRatio; // width/height
    Camera();
    Camera(int focalLength, int imageWidth, int imageHeight,
           Pose3D agentToCameraTransform = Pose3D::Identity());
    Ray3D pixelToWorldRay3D(Vector2i pixel, Pose3D agentPose) const;
    glm::mat4 glmProjectionMatrix() const;
};

#endif // VISUALGROUPCOVERAGEPLANNER_CAMERA_H