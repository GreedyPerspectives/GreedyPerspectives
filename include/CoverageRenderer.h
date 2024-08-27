/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_COVERAGERENDERER_H
#define VISUALGROUPCOVERAGEPLANNER_COVERAGERENDERER_H

#include "GLHelpers.h"
#include "Logging.h"
#include "World.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// Rescale value used to downsample renderings
#define COVERAGE_IMAGE_RESCALE 10

using namespace geometry;
using namespace geometry::polygon;
// Container for actor's vertex array object
struct ActorVertexArray
{
    GLuint vao;
    const Actor & _actor;
    std::vector<float> vertices;
    ActorVertexArray(const Actor & actor, GLint posAttrib, GLint colorAttrib);
};
class CoverageRenderer
{
private:
    // Look at https://open.gl for tutorial explanation
    // TODO move these gl object ints into generalized class
    World & _world;
    // Shader programs
    GLuint shaderProgram;
    GLuint mapShaderProgram;
    // Model view project matrix uniforms
    GLint uniView;
    GLint uniProj;
    GLint uniModel;
    GLint uniModelMap;
    GLint uniViewMap;
    GLint uniProjMap;
    // Vertex array objects
    GLuint vaoMap;
    std::vector<ActorVertexArray> actorVaos;

    // used for rendering the internal image
    GLFWwindow * window;

public:
    const int windowWidth;
    const int windowHeight;
    CoverageRenderer(World & world, bool picking = true);
    ~CoverageRenderer();
    // when rendering coverage directly first create headless window
    void createHeadlessWindow();
    // clean up window when rendering headless
    void destroyHeadlessWindow();
    // initializes all opengl objects for coverage
    void glInit();
    // draws a specific agent's view to the current buffer
    void glDrawAgentView(int id, int time, Pose3D agentPose);
    // cleanup opengl objects
    void glCleanup();
    // get the agent's view as a coverage
    CoverageImage agentCoverageImage(int id, int time); // for after trajectory is set
    CoverageImage agentCoverageImage(int id, State agentState);
    CoverageImage agentCoverageImage(int id, int time, Pose3D agentPose);
};

#endif // VISUALGROUPCOVERAGEPLANNER_COVERAGERENDERER_H