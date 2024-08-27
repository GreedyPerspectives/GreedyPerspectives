/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/
#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_BEVRENDERER_H
#define VISUALGROUPCOVERAGEPLANNER_BEVRENDERER_H

#include "GLHelpers.h"
#include "Logging.h"
#include "World.h"
#include <Eigen/Dense>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <blend2d.h>

using namespace geometry;

// Bird's Eye View renderer for the World class.
class BEVRenderer
{
private:
    int _previousDrawTime = -1;
    const World & _world;
    BLImage img;
    BLContext ctx;
    BLImageData * imgData;
    GLuint _tex;
    GLuint _shaderProgram;
    GLuint _vao;
    GLuint _vbo;
    GLuint _ebo;
    void drawLine(Vector2d start, Vector2d end, BLRgba32 color = BLRgba32(0, 0, 0, 255),
                  double strokeWidth = 3);
    void drawAgent(const Agent & agent, int time);
    void drawActor(const Actor & actor, int time);
    void drawCameraView(const Camera & camera, Pose3D agentPose);
    void drawMap();
    void clearImage();

public:
    float _worldToPixels;
    BEVRenderer(World & world, int windowWidth, int windowHeight);

    void draw(int time, bool cache = true);
    void saveImage(int time, std::string name = "bev.png");

    void glInit();
    void glDraw(int time);
    // Delete opengl
    void glCleanup();
};

#endif // VISUALGROUPCOVERAGEPLANNER_BEVRENDERER_H
