/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_VISUALIZER_H
#define VISUALGROUPCOVERAGEPLANNER_VISUALIZER_H

#include "BEVRenderer.h"
#include "CoverageRenderer.h"
#include "Logging.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl/imgui_impl_glfw.h>
#include <imgui_impl/imgui_impl_opengl3.h>

#define WINDOW_HEIGHT 1000
#define WINDOW_WIDTH 1800

/**
 * Example usage:
 *   Visualizer vis = Visualizer(world);
 *   vis.initializeRenderers();
 *   vis.displayLoop();
 *   vis.cleanupRenderers();
 */

class Visualizer
{
private:
    GLFWwindow * window;
    World & _world;
    BEVRenderer _bev;
    CoverageRenderer _coverage;

    static float w2P;
    static void mouseCallback(GLFWwindow* window, int button, int action, int mods);
public:
    // Init GLFW, glew, imgui, opengl and start the window/context
    Visualizer(World & world);

    // Clean up windows and opengl objects
    ~Visualizer();

    // Call the bev and coverage initialize functions. always paired with cleanup
    void initializeRenderers();

    // Call renderer cleanups, always paired with initialize
    void cleanupRenderers();

    // rendering loop to display the visualization to the screen, called after init
    void displayLoop();
};

#endif // VISUALGROUPCOVERAGEPLANNER_VISUALIZER_H
