/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#include "Camera.h"

int Camera::cameraCount = 0;

Camera::Camera()
  : Camera(1865, 4000, 3000, Pose3D::Identity())
{
    // clang-format off
    _cameraIntrinsicsMatrix << _focalLength, 0, _imageWidth / 2, 
                                0,_focalLength,_imageHeight / 2, 
                                0, 0, 1;
    double camTilt = M_PI / 2 - DEFAULT_CAMERA_TILT;
    // clang-format on

    pose3d::rotateX(_agentToCameraTransform, camTilt);
};

Camera::Camera(int focalLength, int imageWidth, int imageHeight,
               Pose3D agentToCameraTransform)
  : _agentToCameraTransform(agentToCameraTransform)
  , _focalLength(focalLength)
  , _imageWidth(imageWidth)
  , _imageHeight(imageHeight)
  , _id(++cameraCount)
  , _fovY(2.0f * atan(0.5f * _imageHeight / _focalLength) * 180.0f / M_PI)
  , _aspectRatio(_imageWidth / _imageHeight)
{
    // clang-format off
    _cameraIntrinsicsMatrix << _focalLength, 0, _imageWidth / 2, 
                                0,_focalLength,_imageHeight / 2, 
                                0, 0, 1;
    // clang-format on
    float _fovX = 2.0f * atan(0.5f * _imageWidth / _focalLength) * 180.0f / M_PI;
    spdlog::debug("Camera [{}] created with intrinsics (fx,fy,cx,cy): {},{},{},{}",
                  _id,
                  _cameraIntrinsicsMatrix(0, 0),
                  _cameraIntrinsicsMatrix(1, 1),
                  _cameraIntrinsicsMatrix(0, 2),
                  _cameraIntrinsicsMatrix(1, 2));
    spdlog::debug("FOV X: {:.3f} deg, FOV Y: {:.3f} deg, Aspect Ratio {:.3f}",
                  _fovX,
                  _fovY,
                  _aspectRatio);
};

Ray3D Camera::pixelToWorldRay3D(Vector2i pixel, Pose3D agentPose) const
{
    // TODO something is wrong in this math
    double cx = _cameraIntrinsicsMatrix(0, 2);
    double cy = _cameraIntrinsicsMatrix(1, 2);
    Pose3D cameraPoseInWorld = agentPose * _agentToCameraTransform;
    Vector3d pixelHomogeneous(pixel.x() - cx, pixel.y() - cy, -1.0);
    Vector3d direction = _cameraIntrinsicsMatrix.inverse() * pixelHomogeneous;
    direction.normalize();
    return {cameraPoseInWorld.translation(), cameraPoseInWorld.rotation() * direction};
};
glm::mat4 Camera::glmProjectionMatrix() const
{
    return glm::perspective(
        glm::radians((float)_fovY), (float)_aspectRatio, NEAR_PLANE, FAR_PLANE);
};
